#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from whisper_msgs.action import STT
from std_msgs.msg import String  # 导入String消息类型

class WhisperDemoNode(Node):

    def __init__(self) -> None:
        super().__init__("whisper_demo_node")

        self._action_client = ActionClient(self, STT, "/whisper/listen")
        self._publisher = self.create_publisher(String, 'whisper_transcription', 10)  # 创建发布者

    def listen(self) -> None:
        try:
            while True:
                    goal = STT.Goal()

                    self._action_client.wait_for_server()
                    send_goal_future = self._action_client.send_goal_async(goal)

                    rclpy.spin_until_future_complete(self, send_goal_future)
                    get_result_future = send_goal_future.result().get_result_async()
                    self.get_logger().info("SPEAK")

                    rclpy.spin_until_future_complete(self, get_result_future)
                    result: STT.Result = get_result_future.result().result
                    self.get_logger().info(f"I hear: {result.transcription.text}")
                    self.get_logger().info(f"Audio time: {result.transcription.audio_time}")
                    self.get_logger().info(
                        f"Transcription time: {result.transcription.transcription_time}"
                    )

                    # 发布听到的字符串
                    msg = String()
                    msg.data = result.transcription.text
                    self._publisher.publish(msg)

                    # 根据听到的指令调整循环时间
                    if result.transcription.text.lower() in [
                        "forward", "- forward", "go ahead", "go straight", "go forward", "straight", "straight.", "straight", 
                        "backward", "go back", "go backward", "turn left", "left", "left.", "left", 
                        "turn right", "right", "right.", "right"
                    ]:
                        time.sleep(6)
                    else:
                        time.sleep(1)
        except KeyboardInterrupt:
                self.get_logger().info("Exiting...")


def main():

    rclpy.init()
    node = WhisperDemoNode()
    node.listen()
    rclpy.shutdown()

if __name__ == "__main__":
    main()