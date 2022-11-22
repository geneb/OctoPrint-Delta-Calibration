#coding=utf-8
from __future__ import absolute_import

import octoprint.plugin
import octoprint.server

class DeltaCalPlugin(octoprint.plugin.AssetPlugin,
                            octoprint.plugin.TemplatePlugin):
    def on_after_startup(self):
        self._logger.info("Started up!")
		
    def get_assets(self):
        return dict(
            js=["js/deltaautocal.js"]
        )

    def get_template_configs(self):
        return [
            dict(type="settings", template="delta_cal_settings.jinja2", custom_bindings=True)
        ]

    def get_update_information(self):
        return dict(
            systemcommandeditor=dict(
                displayName="Delta Calibration Plugin",
                displayVersion=self._plugin_version,

                # version check: github repository
                type="github_release",
                user="geneb",
                repo="OctoPrint-Delta-Calibration",
                current=self._plugin_version,

                # update method: pip
                pip="https://github.com/geneb/OctoPrint-Delta-Calibration/archive/{target_version}.zip"
            )
        )

__plugin_name__ = "Delta Autocalibration"

__plugin_pythoncompat__ = ">=3,<4"

def __plugin_load__():
    global __plugin_implementation__
    __plugin_implementation__ = DeltaCalPlugin()

    global __plugin_hooks__
    __plugin_hooks__ = {
        "octoprint.plugin.softwareupdate.check_config": __plugin_implementation__.get_update_information
}
