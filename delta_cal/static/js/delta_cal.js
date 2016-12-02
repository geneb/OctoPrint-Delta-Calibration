$(function() {
    //"use strict";
    function DeltaAutoCalViewModel(parameters) {
        var self = this;
        self.control = parameters[0];
        self.connection = parameters[1];

        self.firmwareRegEx = /FIRMWARE_NAME:([^\s]+)/i;
        self.repetierRegEx = /Repetier_([^\s]*)/i;

        self.eepromDataRegEx = /EPR:(\d+) (\d+) ([^\s]+) (.+)/;

        self.M119RegExMinH = "";
        self.M119RegExMaxH = "";
        self.M119RegExMinL= "";
        self.M119RegExMaxL= "";

        self.isRepetierFirmware = ko.observable(false);

        self.isConnected = ko.computed(function() {
            return self.connection.isOperational() || self.connection.isPrinting() ||
                   self.connection.isReady() || self.connection.isPaused();
        });

        self.eepromData = ko.observableArray([]);

        self.statusMessage = ko.observable("");

        self.currentAxis = "";
        self.currentInterval = 0;
        self.currentIteration = 0;
        self.calibrationResult = [];
        self.calibrationStepSize = 0.01;

        self.extruderOffset = [];

        self.onStartup = function() {
            $('#settings_plugin_delta_cal_link a').on('show', function(e) {
                if (self.isConnected() && !self.isRepetierFirmware())
                    self._requestFirmwareInfo();
            });
        }

        self.fromHistoryData = function(data) {
            _.each(data.logs, function(line) {
                var match = self.firmwareRegEx.exec(line);
                if (match != null) {
                    if (self.repetierRegEx.exec(match[0]))
                        self.isRepetierFirmware(true);
                }
            });
        };

        self.fromCurrentData = function(data) {
            if (!self.isRepetierFirmware()) {
                _.each(data.logs, function (line) {
                    var match = self.firmwareRegEx.exec(line);
                    if (match) {
                        if (self.repetierRegEx.exec(match[0]))
                            self.isRepetierFirmware(true);
                    }
                });
            }
            else
            {
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
                    //ugly workaround...
                    //check endstop status and call next iteration
                    if(self.M119RegExMinH != "" && self.currentAxis != "") {
                        if (new RegExp(self.M119RegExMinH).test(line)) {
                            self._calibrationStep(1, true);
                        }
                        if (new RegExp(self.M119RegExMaxH).test(line)) {
                            self._calibrationStep(-1, true);
                        }
                        if (new RegExp(self.M119RegExMinL).test(line)) {
                            self._calibrationStep(1, false);
                        }
                        if (new RegExp(self.M119RegExMaxL).test(line)) {
                            self._calibrationStep(-1, false);
                        }
                    }
                });
            }
        };

        self.onEventConnected = function() {
            self._requestFirmwareInfo();
        }

        self.onEventDisconnected = function() {
            self.isRepetierFirmware(false);
        };

        self.calibrateX = function() {
            self._calibrate("X");
        }
        self.calibrateY = function() {
            self._calibrate("Y");
        }
        self.calibrateZ = function() {
            self._calibrate("Z");
        }

        self._calibrate = function(axis) {
            //fetch current values
            self.statusMessage("Fetching eeprom data");
            self.loadEeprom();
            //move to endstop
            self.currentAxis = axis;
            self.currentInterval = 0.0;

            setTimeout(function() {self._calibrateIteration();}, 5000);
        }

        self._calibrateIteration = function() {
            if(self.currentAxis != "") {
                self.statusMessage("Run calibration iteration " + (self.currentIteration+1));
            
                if(self.currentIteration == 0) {
                    //set calibration to 0
                    self._setEepromValue(self.currentAxis + " backlash", 0.0);

                    self.M119RegExMinH = self.currentAxis.toLowerCase() + "_min:H";
                    self.M119RegExMaxH = self.currentAxis.toLowerCase() + "_max:H";
                    self.M119RegExMinL = self.currentAxis.toLowerCase() + "_min:L";
                    self.M119RegExMaxL = self.currentAxis.toLowerCase() + "_max:L";
                    //Recv: x_min:H y_max:H z_max:H

                    //store extruder offsets
                    var i = 1;
                    do {
                        var offset = self._getEepromValue("Extr." + i + " " + self.currentAxis + "-offset");
                        if(offset) {
                            self.extruderOffset.push(offset);
                            self._setEepromValue("Extr." + i + " " + self.currentAxis + "-offset", 0);
                        }
                        i++;
                    }while(offset);

                    self.saveEeprom();

                    //relative positioning
                    self.control.sendCustomCommand({ command: "G91"});
                    //home axis
                    self.control.sendCustomCommand({ command: "G28 " + self.currentAxis + "0" });
                    self.control.sendCustomCommand({ command: "M400"});
                    self.control.sendCustomCommand({ command: "M400"});
                    self.control.sendCustomCommand({ command: "M400"});
                    self.control.sendCustomCommand({ command: "M400"});
                    self.control.sendCustomCommand({ command: "M400"});
                    self.control.sendCustomCommand({ command: "G4 P0"})
                    self.currentIteration +=1;

                    //trigger endstop check
                    self.control.sendCustomCommand({ command: "M119"});
                }
                else if(2 >= self.currentIteration > 0) {
                    self.control.sendCustomCommand({ command: "G28 " + self.currentAxis + "0" });
                    self.control.sendCustomCommand({ command: "M400"});
                    self.control.sendCustomCommand({ command: "M400"});
                    self.control.sendCustomCommand({ command: "M400"});
                    self.control.sendCustomCommand({ command: "M400"});
                    self.control.sendCustomCommand({ command: "M400"});
                    self.control.sendCustomCommand({ command: "G4 P0"});
                    //trigger endstop check
                    self.currentIteration +=1;
                    self.control.sendCustomCommand({ command: "M119"});
                }
                else if(self.currentIteration > 2) {
                    var newBacklash = 0;
                    //average results
                    var results = self.calibrationResult;
                    for(var i = 0; i < results.length; i++) {
                        newBacklash += results[i];
                    }
                    newBacklash = newBacklash/results.length;
                    newBacklash = Math.round((newBacklash) * 10000) / 10000;
                    self._setEepromValue(self.currentAxis + " backlash", newBacklash);

                    //restore extruder offset
                    for(var i = 0; i < self.extruderOffset.length; i++) {
                        self._setEepromValue("Extr." + (i+1) + " " + self.currentAxis + "-offset", self.extruderOffset[i]);
                    }

                    self.saveEeprom();
                    self.currentAxis = "";
                    self.M119RegExMinH = "";
                    self.M119RegExMaxH = "";
                    self.M119RegExMinL = "";
                    self.M119RegExMaxL = "";
                    self.currentIteration = 0;
                    self.statusMessage("Set backlash to " + newBacklash);
                    self.calibrationResult = [];
                    self.extruderOffset = [];
                    //absolute positioning
                    self.control.sendCustomCommand({ command: "G90"});
                }
            }
        }

        self._calibrationStep = function(sign, endstopStatus) {
            if(endstopStatus) { //endstop still triggered, keep moving
                self.control.sendCustomCommand({ command: "G1 " + self.currentAxis + sign*self.calibrationStepSize + " F500"});
                self.currentInterval += self.calibrationStepSize;
                self.control.sendCustomCommand({ command: "M400" });
                self.control.sendCustomCommand({ command: "G4 P0" });
                self.control.sendCustomCommand({ command: "M119" });
                self.statusMessage(self.statusMessage() + ".");
            }else{ //endstop untriggered, found maximum
                //store result
                self.calibrationResult.push(self.currentInterval-self.calibrationStepSize);
                self.currentInterval = 0;
                self._calibrateIteration();
            }
        }

        self.loadEeprom = function() {
            self.eepromData([]);
            self._requestEepromData();
        };

        self.saveEeprom = function()  {
            var eepromData = self.eepromData();
            _.each(eepromData, function(data) {
                if (data.origValue != data.value) {
                    self._requestSaveDataToEeprom(data.dataType, data.position, data.value);
                    data.origValue = data.value;
                }
            });
        };

        self._getEepromValue = function(description) {
            var eepromData = self.eepromData();
            var result = false;
            _.each(eepromData, function(data) {
                if ((new RegExp(description)).test(data.description)) {
                    result = data.value;
                }
            });
            return result;
        }

        self._setEepromValue = function(description, value) {
            var eepromData = self.eepromData();
            var result = false;
            _.each(eepromData, function(data) {
                if ((new RegExp(description)).test(data.description)) {
                    data.value = value;
                }
            });
        }

        self._requestFirmwareInfo = function() {
            self.control.sendCustomCommand({ command: "M115" });
        };

        self._requestEepromData = function() {
            self.control.sendCustomCommand({ command: "M205" });
        }
        self._requestSaveDataToEeprom = function(data_type, position, value) {
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