$(function () {
    "use strict";
    function DeltaAutoCalViewModel(parameters) {
        var self = this;
        self.control = parameters[0];
        self.connection = parameters[1];

        // machine IDs (these match the SeeMeCNC fork of Repetier).
        const SMC_ORION = 1;
        const SMC_MAX_V2 = 2;
        const SMC_ERIS = 3;
        const SMC_MAX_V3 = 5;  // we skipped 4 because the DropLit is an SLA printer. :)
        const DEFAULT_PROBE_HEIGHT = 25; 

        self.machineType = 0;

        self.firmwareRegEx = /FIRMWARE_NAME:([^\s]+)/i;
        self.repetierRegEx = /Repetier_([^\s]*)/i;

        self.eepromDataRegEx = /EPR:(\d+) (\d+) ([^\s]+) (.+)/;

        self.M119RegExMinH = "";
        self.M119RegExMaxH = "";
        self.M119RegExMinL = "";
        self.M119RegExMaxL = "";

        // this creates functions that can be set here in code and can be referenced
        // externally via the jinja2 file.
        self.isRepetierFirmware = ko.observable(false);
        self.isSeeMeCNCPrinter = ko.observable(false);

        self.eepromData = ko.observableArray([]);

        self.statusMessage = ko.observable("");
        self.statusDebug = ko.observable("");
        self.statusM665 = ko.observable("");
        self.statusM666 = ko.observable("");

        // Delta Calibration variables.
        self.sentM114 = false;
        self.probingActive = false;
        self.probeHot = false;
        self.probeCount = 0;  // so we can keep track of what probe iteration we're on.
        self.commandText = "";  // where the commands to fix things will go for display purposes.

        // dc42 code
        var initialPoints = 7;
        var initialFactors = 6;
        var deltaParams;
        var firmware;
        var bedRadius;
        var numPoints, numFactors;
        var normalise = true; // default from the webpage
        var xBedProbePoints = [];
        var yBedProbePoints = [];
        var zBedProbePoints = [];

        // these are used soley to populate those bits used by the setParameters() routine.
        var oldRodLength = 0;
        var oldRadius = 0;
        var oldHomedHeight = 0;
        var oldXStop = 0;
        var oldYStop = 0;
        var oldZStop = 0;
        var oldXPos = 0;
        var oldYPos = 0;
        var oldZPos = 0;
        var stepsPerMM = 0;

        var newXStop = 0.0;
        var newYStop = 0.0;
        var newZStop = 0.0;
        var newRodLength = 0.0;
        var newRadius = 0.0;
        var newHomedHeight = 0.0;
        var newXPos = 0.0;
        var newYPos = 0.0;
        var newZPos = 0.0;

        var degreesToRadians = Math.PI / 180.0;

        function fsquare(x) {
            return x * x;
        }

        var Matrix = function (rows, cols) {
            this.data = [];
            for (var i = 0; i < rows; ++i) {
                var row = [];
                for (var j = 0; j < cols; ++j) {
                    row.push(0.0);
                }
                this.data.push(row)
            }
        }

        Matrix.prototype.SwapRows = function (i, j, numCols) {
            if (i != j) {
                for (var k = 0; k < numCols; ++k) {
                    var temp = this.data[i][k];
                    this.data[i][k] = this.data[j][k];
                    this.data[j][k] = temp;
                }
            }
        }

        // Perform Gauus-Jordan elimination on a matrix with numRows rows and (njumRows + 1) columns
        Matrix.prototype.GaussJordan = function (solution, numRows) {
            for (var i = 0; i < numRows; ++i) {
                // Swap the rows around for stable Gauss-Jordan elimination
                var vmax = Math.abs(this.data[i][i]);
                for (var j = i + 1; j < numRows; ++j) {
                    var rmax = Math.abs(this.data[j][i]);
                    if (rmax > vmax) {
                        this.SwapRows(i, j, numRows + 1);
                        vmax = rmax;
                    }
                }

                // Use row i to eliminate the ith element from previous and subsequent rows
                var v = this.data[i][i];
                for (var j = 0; j < i; ++j) {
                    var factor = this.data[j][i] / v;
                    this.data[j][i] = 0.0;
                    for (var k = i + 1; k <= numRows; ++k) {
                        this.data[j][k] -= this.data[i][k] * factor;
                    }
                }

                for (var j = i + 1; j < numRows; ++j) {
                    var factor = this.data[j][i] / v;
                    this.data[j][i] = 0.0;
                    for (var k = i + 1; k <= numRows; ++k) {
                        this.data[j][k] -= this.data[i][k] * factor;
                    }
                }
            }

            for (var i = 0; i < numRows; ++i) {
                solution.push(this.data[i][numRows] / this.data[i][i]);
            }
        }

        Matrix.prototype.Print = function (tag) {
            var rslt = tag + " {<br/>";
            for (var i = 0; i < this.data.length; ++i) {
                var row = this.data[i];
                rslt += (row == 0) ? '{' : ' ';
                for (var j = 0; j < row.length; ++j) {
                    rslt += row[j].toFixed(4);
                    if (j + 1 < row.length) {
                        rslt += ", ";
                    }
                }
                rslt += '<br/>';
            }
            rslt += '}';
            return rslt;
        }

        var DeltaParameters = function (diagonal, radius, height, xstop, ystop, zstop, xadj, yadj, zadj) {
            this.diagonal = diagonal;
            this.radius = radius;
            this.homedHeight = height;
            this.xstop = xstop;
            this.ystop = ystop;
            this.zstop = zstop;
            this.xadj = xadj;
            this.yadj = yadj;
            this.zadj = zadj;
            this.Recalc();
        }

        DeltaParameters.prototype.Transform = function (machinePos, axis) {
            return machinePos[2] + Math.sqrt(this.D2 - fsquare(machinePos[0] - this.towerX[axis]) - fsquare(machinePos[1] - this.towerY[axis]));
        }

        // Inverse transform method, We only need the Z component of the result.
        DeltaParameters.prototype.InverseTransform = function (Ha, Hb, Hc) {
            var Fa = this.coreFa + fsquare(Ha);
            var Fb = this.coreFb + fsquare(Hb);
            var Fc = this.coreFc + fsquare(Hc);

            // Setup PQRSU such that x = -(S - uz)/P, y = (P - Rz)/Q
            var P = (this.Xbc * Fa) + (this.Xca * Fb) + (this.Xab * Fc);
            var S = (this.Ybc * Fa) + (this.Yca * Fb) + (this.Yab * Fc);

            var R = 2 * ((this.Xbc * Ha) + (this.Xca * Hb) + (this.Xab * Hc));
            var U = 2 * ((this.Ybc * Ha) + (this.Yca * Hb) + (this.Yab * Hc));

            var R2 = fsquare(R), U2 = fsquare(U);

            var A = U2 + R2 + this.Q2;
            var minusHalfB = S * U + P * R + Ha * this.Q2 + this.towerX[0] * U * this.Q - this.towerY[0] * R * this.Q;
            var C = fsquare(S + this.towerX[0] * this.Q) + fsquare(P - this.towerY[0] * this.Q) + (fsquare(Ha) - this.D2) * this.Q2;

            var rslt = (minusHalfB - Math.sqrt(fsquare(minusHalfB) - A * C)) / A;
            if (isNaN(rslt)) {
                throw "At least one probe point is not reachable. Please correct your delta radius, diagonal rod length, or probe coordinates."
            }
            return rslt;
        }

        DeltaParameters.prototype.Recalc = function () {
            this.towerX = [];
            this.towerY = [];
            this.towerX.push(-(this.radius * Math.cos((30 + this.xadj) * degreesToRadians)));
            this.towerY.push(-(this.radius * Math.sin((30 + this.xadj) * degreesToRadians)));
            this.towerX.push(+(this.radius * Math.cos((30 - this.yadj) * degreesToRadians)));
            this.towerY.push(-(this.radius * Math.sin((30 - this.yadj) * degreesToRadians)));
            this.towerX.push(-(this.radius * Math.sin(this.zadj * degreesToRadians)));
            this.towerY.push(+(this.radius * Math.cos(this.zadj * degreesToRadians)));

            this.Xbc = this.towerX[2] - this.towerX[1];
            this.Xca = this.towerX[0] - this.towerX[2];
            this.Xab = this.towerX[1] - this.towerX[0];
            this.Ybc = this.towerY[2] - this.towerY[1];
            this.Yca = this.towerY[0] - this.towerY[2];
            this.Yab = this.towerY[1] - this.towerY[0];
            this.coreFa = fsquare(this.towerX[0]) + fsquare(this.towerY[0]);
            this.coreFb = fsquare(this.towerX[1]) + fsquare(this.towerY[1]);
            this.coreFc = fsquare(this.towerX[2]) + fsquare(this.towerY[2]);
            this.Q = 2 * (this.Xca * this.Yab - this.Xab * this.Yca);
            this.Q2 = fsquare(this.Q);
            this.D2 = fsquare(this.diagonal);

            // Calculate the base carriage height when the printer is homed.
            var tempHeight = this.diagonal;		// any sensible height will do here, probably even zero
            this.homedCarriageHeight = this.homedHeight + tempHeight - this.InverseTransform(tempHeight, tempHeight, tempHeight);
        }

        DeltaParameters.prototype.ComputeDerivative = function (deriv, ha, hb, hc) {
            var perturb = 0.2;			// perturbation amount in mm or degrees
            var hiParams = new DeltaParameters(this.diagonal, this.radius, this.homedHeight, this.xstop, this.ystop, this.zstop, this.xadj, this.yadj, this.zadj);
            var loParams = new DeltaParameters(this.diagonal, this.radius, this.homedHeight, this.xstop, this.ystop, this.zstop, this.xadj, this.yadj, this.zadj);
            switch (deriv) {
                case 0:
                case 1:
                case 2:
                    break;

                case 3:
                    hiParams.radius += perturb;
                    loParams.radius -= perturb;
                    break;

                case 4:
                    hiParams.xadj += perturb;
                    loParams.xadj -= perturb;
                    break;

                case 5:
                    hiParams.yadj += perturb;
                    loParams.yadj -= perturb;
                    break;

                case 6:
                    hiParams.diagonal += perturb;
                    loParams.diagonal -= perturb;
                    break;
            }

            hiParams.Recalc();
            loParams.Recalc();

            var zHi = hiParams.InverseTransform((deriv == 0) ? ha + perturb : ha, (deriv == 1) ? hb + perturb : hb, (deriv == 2) ? hc + perturb : hc);
            var zLo = loParams.InverseTransform((deriv == 0) ? ha - perturb : ha, (deriv == 1) ? hb - perturb : hb, (deriv == 2) ? hc - perturb : hc);

            return (zHi - zLo) / (2 * perturb);
        }

        // Make the average of the endstop adjustments zero, or make all emndstop corrections negative, without changing the individual homed carriage heights
        DeltaParameters.prototype.NormaliseEndstopAdjustments = function () {
            var eav = (firmware == "Marlin" || firmware == "MarlinRC" || firmware == "Repetier") ? Math.min(this.xstop, Math.min(this.ystop, this.zstop))
                : (this.xstop + this.ystop + this.zstop) / 3.0;
            this.xstop -= eav;
            this.ystop -= eav;
            this.zstop -= eav;
            this.homedHeight += eav;
            this.homedCarriageHeight += eav;				// no need for a full recalc, this is sufficient
        }

        // Perform 3, 4, 6 or 7-factor adjustment.
        // The input vector contains the following parameters in this order:
        //  X, Y and Z endstop adjustments
        //  If we are doing 4-factor adjustment, the next argument is the delta radius. Otherwise:
        //  X tower X position adjustment
        //  Y tower X position adjustment
        //  Z tower Y position adjustment
        //  Diagonal rod length adjustment
        DeltaParameters.prototype.Adjust = function (numFactors, v, norm) {
            var oldCarriageHeightA = this.homedCarriageHeight + this.xstop;	// save for later

            // Update endstop adjustments
            this.xstop += v[0];
            this.ystop += v[1];
            this.zstop += v[2];
            if (norm) {
                this.NormaliseEndstopAdjustments();
            }

            if (numFactors >= 4) {
                this.radius += v[3];

                if (numFactors >= 6) {
                    this.xadj += v[4];
                    this.yadj += v[5];

                    if (numFactors == 7) {
                        this.diagonal += v[6];
                    }
                }

                this.Recalc();
            }

            // Adjusting the diagonal and the tower positions affects the homed carriage height.
            // We need to adjust homedHeight to allow for this, to get the change that was requested in the endstop corrections.
            var heightError = this.homedCarriageHeight + this.xstop - oldCarriageHeightA - v[0];
            this.homedHeight -= heightError;
            this.homedCarriageHeight -= heightError;
        }

        function PrintVector(label, v) {
            var rslt = label + ": {";
            for (var i = 0; i < v.length; ++i) {
                rslt += v[i].toFixed(4);
                if (i + 1 != v.length) {
                    rslt += ", ";
                }
            }
            rslt += "}";
            return rslt;
        }


        function calcProbePoints() {
            // This function produces a set of recommended probing points based on initial geometry values
            // obtained from the web page via getParameters.
            //numPoints = +document.getElementById("numPoints").value;  // Number of points to probe
            //bedRadius = +document.getElementById("bedradius").value;  // Bed radius.
            if (numPoints == 4) {
                for (var i = 0; i < 3; ++i) {
                    xBedProbePoints[i] = (bedRadius * Math.sin((2 * Math.PI * i) / 3)).toFixed(2);
                    yBedProbePoints[i] = (bedRadius * Math.cos((2 * Math.PI * i) / 3)).toFixed(2);
                    zBedProbePoints[i] = 0.0; // we default this to zero -gwb
                }
                xBedProbePoints[3] = 0.0;
                yBedProbePoints[3] = 0.0;
                zBedProbePoints[3] = 0.0;
            }
            else {
                if (numPoints >= 7) {
                    for (var i = 0; i < 6; ++i) {
                        xBedProbePoints[i] = (bedRadius * Math.sin((2 * Math.PI * i) / 6)).toFixed(2);
                        yBedProbePoints[i] = (bedRadius * Math.cos((2 * Math.PI * i) / 6)).toFixed(2);
                        zBedProbePoints[i] = 0.0; // we default this to zero -gwb
                    }
                }
                if (numPoints >= 10) {
                    for (var i = 6; i < 9; ++i) {
                        xBedProbePoints[i] = (bedRadius / 2 * Math.sin((2 * Math.PI * (i - 6)) / 3)).toFixed(2);
                        yBedProbePoints[i] = (bedRadius / 2 * Math.cos((2 * Math.PI * (i - 6)) / 3)).toFixed(2);
                        zBedProbePoints[i] = 0.0; // we default this to zero -gwb
                    }
                    xBedProbePoints[9] = 0.0;
                    yBedProbePoints[9] = 0.0;
                    zBedProbePoints[9] = 0.0;
                }
                else {
                    xBedProbePoints[6] = 0.0;
                    yBedProbePoints[6] = 0.0;
                    zBedProbePoints[6] = 0.0;
                }
            }
        }

        self.isConnected = ko.computed(function () {
            return self.connection.isOperational() || self.connection.isPrinting() ||
                self.connection.isReady() || self.connection.isPaused();
        });

        function setParameters() {
            // this is kind of a kludge in order to get the routine working with a minimal amount of hackery.
            switch (self.machineType) {
                case SMC_ORION: {
                    stepsPerMM = 80;
                    bedRadius = 90;

                    oldRodLength = 178.0; // based on the ball-cup arms
                    oldRadius = 145.7;
                    oldHomedHeight = 230.0; // max Z height.
                    oldXStop = 0;
                    oldYStop = 0;
                    oldZStop = 0;
                    oldXPos = 0;
                    oldYPos = 0;
                    oldZPos = 0;
                    calcProbePoints();
                }
                case SMC_MAX_V2: {
                    stepsPerMM = 80;
                    bedRadius = 91; // Throw's NaNs higher than 91! 140;

                    oldRodLength = 291.06; // based on the ball-cup arms
                    oldRadius = 200.0;
                    oldHomedHeight = 350.0; // max Z height.
                    oldXStop = 0;
                    oldYStop = 0;
                    oldZStop = 0;
                    oldXPos = 0;
                    oldYPos = 0;
                    oldZPos = 0;
                    calcProbePoints();
                }
                case SMC_ERIS: {
                    stepsPerMM = 80;
                    bedRadius = 60;

                    oldRodLength = 134.9; // based on the ball-cup arms
                    oldRadius = 98.38;
                    oldHomedHeight = 175.0; // max Z height.
                    oldXStop = 0;
                    oldYStop = 0;
                    oldZStop = 0;
                    oldXPos = 0;
                    oldYPos = 0;
                    oldZPos = 0;
                    calcProbePoints();
                }
                case SMC_MAX_V3: {
                    stepsPerMM = 80;
                    bedRadius = 91; // Throw's NaNs higher than 91! 140;

                    oldRodLength = 291.06; // based on the ball-cup arms
                    oldRadius = 200.0;
                    oldHomedHeight = 375.0; // max Z height.
                    oldXStop = 0;
                    oldYStop = 0;
                    oldZStop = 0;
                    oldXPos = 0;
                    oldYPos = 0;
                    oldZPos = 0;
                    calcProbePoints();
                }
            }
            deltaParams = new DeltaParameters(oldRodLength, oldRadius, oldHomedHeight,
                oldXStop, oldYStop, oldZStop, oldXPos, oldYPos, oldZPos);

        }

        function convertIncomingEndstops() {
            var endstopFactor = (firmware == "RRF") ? 1.0
                : (firmware == "Repetier") ? 1.0 / stepsPerMM
                    : -1.0;
            deltaParams.xstop *= endstopFactor;
            deltaParams.ystop *= endstopFactor;
            deltaParams.zstop *= endstopFactor;
        }

        function convertOutgoingEndstops() {
            var endstopFactor = (firmware == "RRF") ? 1.0
                : (firmware == "Repetier") ? (stepsPerMM)
                    : -1.0;
            deltaParams.xstop *= endstopFactor;
            deltaParams.ystop *= endstopFactor;
            deltaParams.zstop *= endstopFactor;
        }

        function setNewParameters() {
            var endstopPlaces = (firmware == "Repetier") ? 0 : 2;
            newXStop = deltaParams.xstop.toFixed(endstopPlaces);
            newYStop = deltaParams.ystop.toFixed(endstopPlaces);
            newZStop = deltaParams.zstop.toFixed(endstopPlaces);
            newRodLength = deltaParams.diagonal.toFixed(2);
            newRadius = deltaParams.radius.toFixed(2);
            newHomedHeight = deltaParams.homedHeight.toFixed(2);
            newXPos = deltaParams.xadj.toFixed(2);
            newYPos = deltaParams.yadj.toFixed(2);
            newZPos = deltaParams.zadj.toFixed(2);
        }

        self.beginDeltaCal = function () {
            numPoints = 7;  // these should be configurable at some point.
            numFactors = 6;
            firmware = "Repetier"
            // here's where we begin to accumulate the data needed to run the actual calculations.
            setParameters();  // develops our probing points.
            convertIncomingEndstops();

            // kick off the first probe!
            self.probeCount = 0;
            self.probingActive = true;
            self.control.sendCustomCommand({ command: "G28" }); // home first!
            // first probe.
            self.control.sendCustomCommand({
                command: "G0 X" + xBedProbePoints[self.probeCount] +
                " Y" + yBedProbePoints[self.probeCount] + " Z" + DEFAULT_PROBE_HEIGHT + " F6500"
            });

            self.control.sendCustomCommand({ command: "G30" });

        }

        function startDeltaCalcEngine() {
            //numPoints = 7;  // these should be configurable at some point.
            //numFactors = 6;
            //firmware = "Repetier"

            //setParameters();
            //convertIncomingEndstops();
            try {
                var rslt = DoDeltaCalibration();
                //document.getElementById("result").innerHTML = "&nbsp;Success! " + rslt + "&nbsp;";
                //document.getElementById("result").style.backgroundColor = "LightGreen";
                self.statusMessage("Success!");

                convertOutgoingEndstops();
                setNewParameters();
                generateCommands();
                //document.getElementById("copyButton").disabled = false;
                self.probingActive = false; // all done!
            }
            catch (err) {
                self.statusMessage(self.statusMessage() + "Error! - " + err);
                //document.getElementById("result").innerHTML = "&nbsp;Error! " + err + "&nbsp;";
                //document.getElementById("result").style.backgroundColor = "LightPink";
                //document.getElementById("copyButton").disabled = true;
            }
        }

        function generateCommands() {
            var m665 = "M665 R" + deltaParams.radius.toFixed(2) + " L" + deltaParams.diagonal.toFixed(2);
            var m666 = "M666 X" + deltaParams.xstop.toFixed(2) + " Y" + deltaParams.ystop.toFixed(2) + " Z" + deltaParams.zstop.toFixed(2);
            switch (firmware) {
                case 'RRF':
                    m665 += " H" + deltaParams.homedHeight.toFixed(2) + " B" + bedRadius.toFixed(2)
                        + " X" + deltaParams.xadj.toFixed(2) + " Y" + deltaParams.yadj.toFixed(2) + " Z" + deltaParams.zadj.toFixed(2);
                    break;
                case 'Marlin':
                    break;
                case 'MarlinRC':
                    m666 += " R" + deltaParams.radius.toFixed(2) + " D" + deltaParams.diagonal.toFixed(2) + " H" + deltaParams.homedHeight.toFixed(2)
                        + " A" + deltaParams.xadj.toFixed(2) + " B" + deltaParams.yadj.toFixed(2) + " C" + deltaParams.zadj.toFixed(2);
                    break;
                case 'Repetier':
                    break;
                case 'Smoothieware':
                    m665 += " D" + deltaParams.xadj.toFixed(2) + " E" + deltaParams.yadj.toFixed(2) + " H" + deltaParams.zadj.toFixed(2) + " Z" + deltaParams.homedHeight.toFixed(2);
                    break;
            }
            var commands = "";
            if (firmware != 'MarlinRC') {
                commands += m665 + "\n";
            }
            commands += m666;
            if (firmware == 'Marlin') {
                commands += "\n; Set homed height " + deltaParams.homedHeight.toFixed(2) + "mm in config.h";
            }
            self.commandText = commands;
            self.statusM665(m665);
            self.statusM666(m666);
        }



        self.currentAxis = "";
        self.currentInterval = 0;
        self.currentIteration = 0;
        self.calibrationResult = [];
        self.calibrationStepSize = 0.01;

        self.extruderOffset = [];

        self.onStartup = function () {
            $('#settings_plugin_delta_cal_link a').on('show', function (e) {
                if (self.isConnected() && !self.isRepetierFirmware())
                    self._requestFirmwareInfo();
            });
        }

        self.fromHistoryData = function (data) {
            _.each(data.logs, function (line) {
                var match = self.firmwareRegEx.exec(line);
                if (match != null) {
                    if (self.repetierRegEx.exec(match[0]))
                        self.isRepetierFirmware(true);
                }
            });
        };

        self.fromCurrentData = function (data) {
            if (!self.isRepetierFirmware()) {
                _.each(data.logs, function (line) {
                    var match = self.firmwareRegEx.exec(line);
                    if (match) {
                        if (self.repetierRegEx.exec(match[0])) {
                            self.isSeeMeCNCPrinter(false); //.. unless otherwise!
                            if (line.includes("ORION Delta")) {
                                self.machineType = SMC_ORION;
                                self.isSeeMeCNCPrinter(true);
                                self.statusMessage("Orion Delta detected!")
                            }
                            if (line.includes("Rostock Max v2")) {
                                self.machineType = SMC_MAX_V2;
                                self.isSeeMeCNCPrinter(true);
                                self.statusMessage("Rostock Max v2 detected!");
                            }
                            if (line.includes("ERIS Delta")) {
                                self.machineType = SMC_ERIS;
                                self.isSeeMeCNCPrinter(true);
                                self.statusMessage("Eris Delta detected!");
                            }
                            if (line.includes("Rostock MAX v3")) {
                                self.machineType = SMC_MAX_V3;
                                self.isSeeMeCNCPrinter(true);
                                self.statusMessage("Rostock Max v3 detected!");
                            }
                            self.isRepetierFirmware(true);

                        }
                    }
                });
            }
            else {
                _.each(data.logs, function (line) {
                    var match = self.eepromDataRegEx.exec(line);
                    if (match) {
                        self.eepromData.push({
                            dataType: match[1],
                            position: match[2],
                            origValue: match[3],
                            value: match[3],
                            description: match[4]
                        });
                    }
                    if (self.sentM114) {
                        if (line.includes("X") && line.includes("Y") && line.includes("Z") && line.includes("E")) {
                            // we've got the result of an M114 here.
                            self.statusMessage(self.statusMessage() + "M114 Result: " + line);
                            self.sentM114 = false;
                        }
                        // self.statusMessage(self.statusMessage() + "M114 Result: " + line);
                        // self.sentM114 = false;
                    }
                    if (line.includes("zprobing")) {
                        self.probeHot = true;
                    }
                    if (self.probingActive && self.probeHot) {
                        // find the result and show it!
                        if (line.includes("X") && line.includes("Y") && line.includes("Z") && line.includes("E")) {
                            // we've got the result of a probe!
                            var coords = line.split(" ");
                            self.statusDebug(self.statusDebug() + " Probe value: " + coords[3].substring(2));
                            zBedProbePoints[self.probeCount] = parseFloat(coords[3].substring(2));
                            self.probeHot = false;
                            // store the result.
                            //zBedProbePoints[self.probeCount] = ...
                            self.probeCount++;
                            if (self.probeCount == numPoints) {
                                startDeltaCalcEngine();  // doooo eeeeeeet!
                            } else {
                                // probe next point.
                                self.control.sendCustomCommand({
                                    command: "G0 X" + xBedProbePoints[self.probeCount] +
                                    " Y" + yBedProbePoints[self.probeCount] + " Z" + DEFAULT_PROBE_HEIGHT + " F6500"
                                });

                                self.control.sendCustomCommand({ command: "G30" });
                            }
                        }

                    }
                    //ugly workaround...
                    //check endstop status and call next iteration
                    // if (self.M119RegExMinH != "" && self.currentAxis != "") {
                    //     if (new RegExp(self.M119RegExMinH).test(line)) {
                    //         self._calibrationStep(1, true);
                    //     }
                    //     if (new RegExp(self.M119RegExMaxH).test(line)) {
                    //         self._calibrationStep(-1, true);
                    //     }
                    //     if (new RegExp(self.M119RegExMinL).test(line)) {
                    //         self._calibrationStep(1, false);
                    //     }
                    //     if (new RegExp(self.M119RegExMaxL).test(line)) {
                    //         self._calibrationStep(-1, false);
                    //     }
                    // }
                });
            }
        };

        self.onEventConnected = function () {
            self._requestFirmwareInfo();
        }

        self.onEventDisconnected = function () {
            self.isRepetierFirmware(false);
        };

        function DoDeltaCalibration() {
            if (numFactors != 3 && numFactors != 4 && numFactors != 6 && numFactors != 7) {
                self.statusMessage(self.StatusMessage() + "Error: " + numFactors + " factors requested but only 3, 4, 6 and 7 supported");
                return;
            }
            if (numFactors > numPoints) {
                self.statusMessage(self.StatusMessage() + "Error: need at least as many points as factors you want to calibrate");
                return;
            }

            // Transform the probing points to motor endpoints and store them in a matrix, so that we can do multiple iterations using the same data
            var probeMotorPositions = new Matrix(numPoints, 3);
            var corrections = new Array(numPoints);
            var initialSumOfSquares = 0.0;
            for (var i = 0; i < numPoints; ++i) {
                corrections[i] = 0.0;
                var machinePos = [];
                var xp = xBedProbePoints[i], yp = yBedProbePoints[i];
                machinePos.push(xp);
                machinePos.push(yp);
                machinePos.push(0.0);

                probeMotorPositions.data[i][0] = deltaParams.Transform(machinePos, 0);
                probeMotorPositions.data[i][1] = deltaParams.Transform(machinePos, 1);
                probeMotorPositions.data[i][2] = deltaParams.Transform(machinePos, 2);

                initialSumOfSquares += fsquare(zBedProbePoints[i]);
            }

            // Do 1 or more Newton-Raphson iterations
            var iteration = 0;
            var expectedRmsError;
            for (; ;) {
                // Build a Nx7 matrix of derivatives with respect to xa, xb, yc, za, zb, zc, diagonal.
                var derivativeMatrix = new Matrix(numPoints, numFactors);
                for (var i = 0; i < numPoints; ++i) {
                    for (var j = 0; j < numFactors; ++j) {
                        derivativeMatrix.data[i][j] =
                            deltaParams.ComputeDerivative(j, probeMotorPositions.data[i][0], probeMotorPositions.data[i][1], probeMotorPositions.data[i][2]);
                    }
                }

                //DebugPrint(derivativeMatrix.Print("Derivative matrix:"));

                // Now build the normal equations for least squares fitting
                var normalMatrix = new Matrix(numFactors, numFactors + 1);
                for (var i = 0; i < numFactors; ++i) {
                    for (var j = 0; j < numFactors; ++j) {
                        var temp = derivativeMatrix.data[0][i] * derivativeMatrix.data[0][j];
                        for (var k = 1; k < numPoints; ++k) {
                            temp += derivativeMatrix.data[k][i] * derivativeMatrix.data[k][j];
                        }
                        normalMatrix.data[i][j] = temp;
                    }
                    var temp = derivativeMatrix.data[0][i] * -(zBedProbePoints[0] + corrections[0]);
                    for (var k = 1; k < numPoints; ++k) {
                        temp += derivativeMatrix.data[k][i] * -(zBedProbePoints[k] + corrections[k]);
                    }
                    normalMatrix.data[i][numFactors] = temp;
                }

                //DebugPrint(normalMatrix.Print("Normal matrix:"));

                var solution = [];
                normalMatrix.GaussJordan(solution, numFactors);

                for (var i = 0; i < numFactors; ++i) {
                    if (isNaN(solution[i])) {
                        throw "Unable to calculate corrections. Please make sure the bed probe points are all distinct.";
                    }
                }

                // DebugPrint(normalMatrix.Print("Solved matrix:"));

                // if (debug) {
                //     DebugPrint(PrintVector("Solution", solution));

                //     // Calculate and display the residuals
                //     var residuals = [];
                //     for (var i = 0; i < numPoints; ++i) {
                //         var r = zBedProbePoints[i];
                //         for (var j = 0; j < numFactors; ++j) {
                //             r += solution[j] * derivativeMatrix.data[i][j];
                //         }
                //         residuals.push(r);
                //     }
                //     DebugPrint(PrintVector("Residuals", residuals));
                // }

                deltaParams.Adjust(numFactors, solution, normalise);

                // Calculate the expected probe heights using the new parameters
                {
                    var expectedResiduals = new Array(numPoints);
                    var sumOfSquares = 0.0;
                    for (var i = 0; i < numPoints; ++i) {
                        for (var axis = 0; axis < 3; ++axis) {
                            probeMotorPositions.data[i][axis] += solution[axis];
                        }
                        var newZ = deltaParams.InverseTransform(probeMotorPositions.data[i][0], probeMotorPositions.data[i][1], probeMotorPositions.data[i][2]);
                        corrections[i] = newZ;
                        expectedResiduals[i] = zBedProbePoints[i] + newZ;
                        sumOfSquares += fsquare(expectedResiduals[i]);
                    }

                    expectedRmsError = Math.sqrt(sumOfSquares / numPoints);
                    //DebugPrint(PrintVector("Expected probe error", expectedResiduals));
                }

                // Decide whether to do another iteration Two is slightly better than one, but three doesn't improve things.
                // Alternatively, we could stop when the expected RMS error is only slightly worse than the RMS of the residuals.
                ++iteration;
                if (iteration == 2) { break; }
            }

            self.statusDebug(self.statusDebug() + "    Calibrated " + numFactors + " factors using " + numPoints + " points, deviation before " + Math.sqrt(initialSumOfSquares / numPoints).toFixed(2)
                + " after " + expectedRmsError.toFixed(2));

            // // first we home the printer.
            // self.control.sendCustomCommand({ command: "G28" });
            // // now position to our first point...
            // self.control.sendCustomCommand({ command: "G0 X50 Y50 Z50 F6500" });
            // // now we probe!
            // self.control.sendCustomCommand({ command: "G30" });
            // self.probingActive = true;
        }
        ////////////////////////////////////////////////////////////////////////
        // End of dc42's code.
        ////////////////////////////////////////////////////////////////////////


        self.showCoords = function () {
            self.control.sendCustomCommand({ command: "M114" });
            self.statusMessage(self.statusMessage() + "Sent M114.");
            self.sentM114 = true;
        }
        // self.calibrateX = function () {
        //     self._calibrate("X");
        // }
        // self.calibrateY = function () {
        //     self._calibrate("Y");
        // }
        // self.calibrateZ = function () {
        //     self._calibrate("Z");
        // }

        // self._calibrate = function (axis) {
        //     //fetch current values
        //     self.statusMessage("Fetching eeprom data");
        //     self.loadEeprom();
        //     //move to endstop
        //     self.currentAxis = axis;
        //     self.currentInterval = 0.0;

        //     setTimeout(function () { self._calibrateIteration(); }, 5000);
        // }

        // self._calibrateIteration = function () {
        //     if (self.currentAxis != "") {
        //         self.statusMessage("Run calibration iteration " + (self.currentIteration + 1));

        //         if (self.currentIteration == 0) {
        //             //set calibration to 0
        //             self._setEepromValue(self.currentAxis + " backlash", 0.0);

        //             self.M119RegExMinH = self.currentAxis.toLowerCase() + "_min:H";
        //             self.M119RegExMaxH = self.currentAxis.toLowerCase() + "_max:H";
        //             self.M119RegExMinL = self.currentAxis.toLowerCase() + "_min:L";
        //             self.M119RegExMaxL = self.currentAxis.toLowerCase() + "_max:L";
        //             //Recv: x_min:H y_max:H z_max:H

        //             //store extruder offsets
        //             var i = 1;
        //             do {
        //                 var offset = self._getEepromValue("Extr." + i + " " + self.currentAxis + "-offset");
        //                 if (offset) {
        //                     self.extruderOffset.push(offset);
        //                     self._setEepromValue("Extr." + i + " " + self.currentAxis + "-offset", 0);
        //                 }
        //                 i++;
        //             } while (offset);

        //             self.saveEeprom();

        //             //relative positioning
        //             self.control.sendCustomCommand({ command: "G91" });
        //             //home axis
        //             self.control.sendCustomCommand({ command: "G28 " + self.currentAxis + "0" });
        //             self.control.sendCustomCommand({ command: "M400" });
        //             self.control.sendCustomCommand({ command: "M400" });
        //             self.control.sendCustomCommand({ command: "M400" });
        //             self.control.sendCustomCommand({ command: "M400" });
        //             self.control.sendCustomCommand({ command: "M400" });
        //             self.control.sendCustomCommand({ command: "G4 P0" })
        //             self.currentIteration += 1;

        //             //trigger endstop check
        //             self.control.sendCustomCommand({ command: "M119" });
        //         }
        //         else if (2 >= self.currentIteration > 0) {
        //             self.control.sendCustomCommand({ command: "G28 " + self.currentAxis + "0" });
        //             self.control.sendCustomCommand({ command: "M400" });
        //             self.control.sendCustomCommand({ command: "M400" });
        //             self.control.sendCustomCommand({ command: "M400" });
        //             self.control.sendCustomCommand({ command: "M400" });
        //             self.control.sendCustomCommand({ command: "M400" });
        //             self.control.sendCustomCommand({ command: "G4 P0" });
        //             //trigger endstop check
        //             self.currentIteration += 1;
        //             self.control.sendCustomCommand({ command: "M119" });
        //         }
        //         else if (self.currentIteration > 2) {
        //             var newBacklash = 0;
        //             //average results
        //             var results = self.calibrationResult;
        //             for (var i = 0; i < results.length; i++) {
        //                 newBacklash += results[i];
        //             }
        //             newBacklash = newBacklash / results.length;
        //             newBacklash = Math.round((newBacklash) * 10000) / 10000;
        //             self._setEepromValue(self.currentAxis + " backlash", newBacklash);

        //             //restore extruder offset
        //             for (var i = 0; i < self.extruderOffset.length; i++) {
        //                 self._setEepromValue("Extr." + (i + 1) + " " + self.currentAxis + "-offset", self.extruderOffset[i]);
        //             }

        //             self.saveEeprom();
        //             self.currentAxis = "";
        //             self.M119RegExMinH = "";
        //             self.M119RegExMaxH = "";
        //             self.M119RegExMinL = "";
        //             self.M119RegExMaxL = "";
        //             self.currentIteration = 0;
        //             self.statusMessage("Set backlash to " + newBacklash);
        //             self.calibrationResult = [];
        //             self.extruderOffset = [];
        //             //absolute positioning
        //             self.control.sendCustomCommand({ command: "G90" });
        //         }
        //     }
        // }

        // self._calibrationStep = function (sign, endstopStatus) {
        //     if (endstopStatus) { //endstop still triggered, keep moving
        //         self.control.sendCustomCommand({ command: "G1 " + self.currentAxis + sign * self.calibrationStepSize + " F500" });
        //         self.currentInterval += self.calibrationStepSize;
        //         self.control.sendCustomCommand({ command: "M400" });
        //         self.control.sendCustomCommand({ command: "G4 P0" });
        //         self.control.sendCustomCommand({ command: "M119" });
        //         self.statusMessage(self.statusMessage() + ".");
        //     } else { //endstop untriggered, found maximum
        //         //store result
        //         self.calibrationResult.push(self.currentInterval - self.calibrationStepSize);
        //         self.currentInterval = 0;
        //         self._calibrateIteration();
        //     }
        // }

        // self.loadEeprom = function () {
        //     self.eepromData([]);
        //     self._requestEepromData();
        // };

        // self.saveEeprom = function () {
        //     var eepromData = self.eepromData();
        //     _.each(eepromData, function (data) {
        //         if (data.origValue != data.value) {
        //             self._requestSaveDataToEeprom(data.dataType, data.position, data.value);
        //             data.origValue = data.value;
        //         }
        //     });
        // };

        // self._getEepromValue = function (description) {
        //     var eepromData = self.eepromData();
        //     var result = false;
        //     _.each(eepromData, function (data) {
        //         if ((new RegExp(description)).test(data.description)) {
        //             result = data.value;
        //         }
        //     });
        //     return result;
        // }

        // self._setEepromValue = function (description, value) {
        //     var eepromData = self.eepromData();
        //     var result = false;
        //     _.each(eepromData, function (data) {
        //         if ((new RegExp(description)).test(data.description)) {
        //             data.value = value;
        //         }
        //     });
        // }

        self._requestFirmwareInfo = function () {
            self.control.sendCustomCommand({ command: "M115" });
        };

        self._requestEepromData = function () {
            self.control.sendCustomCommand({ command: "M205" });
        }
        self._requestSaveDataToEeprom = function (data_type, position, value) {
            var cmd = "M206 T" + data_type + " P" + position;
            if (data_type == 3) {
                cmd += " X" + value;
                self.control.sendCustomCommand({ command: cmd });
            }
            else {
                cmd += " S" + value;
                self.control.sendCustomCommand({ command: cmd });
            }
        }
    }

    OCTOPRINT_VIEWMODELS.push([
        DeltaAutoCalViewModel,
        ["controlViewModel", "connectionViewModel"],
        "#settings_plugin_delta_cal"
    ]);
});