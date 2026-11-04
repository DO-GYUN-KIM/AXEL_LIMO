import os

import omni.ext
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from .limo_ros2_ack import LimoROS2Ackermann


class LimoROS2AckermannExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.example_name = "Limo ROS2 Ackermann (AXEL)"
        self.category = "Wego"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Limo ROS2 Ackermann (AXEL)",
            "doc_link": "",
            "overview": "LIMO 애커먼 모델 + /sim/ackermann_cmd 구독 + /scan, /odom, 카메라 발행. 배치는 scenario.json",
            "sample": LimoROS2Ackermann(),
        }
        ui_handle = BaseSampleUITemplate(**ui_kwargs)
        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

    def on_shutdown(self):
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
