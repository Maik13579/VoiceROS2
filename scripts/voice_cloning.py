#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from coqui_tts_ros2_interfaces.action import TTS as TTSAction

from pathlib import Path
import sys

def get_wav_files_string(folder: str) -> str:
    """
    Returns a semicolon-separated string of full paths for all .wav files in the specified folder.
    """
    folder_path = Path(folder)
    wav_files = folder_path.glob("*.wav")
    file_list = [str(wav.resolve()) for wav in wav_files]
    return ";".join(file_list)

class GPSR_Demo(Node):
    def __init__(self):
        super().__init__('voice_cloning_demo')
        # Action clients for TTS and Speech Detection
        self.tts_voice_clone_client = ActionClient(self, TTSAction, '/tts_voice_clone/tts')

    def tts_voice_clone(self, text, language='en', speaker_wav='', wait_before_speaking=0.0):
        goal_msg = TTSAction.Goal()
        goal_msg.text = text
        goal_msg.language = language
        goal_msg.speaker_wav = speaker_wav
        goal_msg.wait_before_speaking = wait_before_speaking

        self.get_logger().info('Sending TTS goal...')
        self.tts_voice_clone_client.wait_for_server()
        return self.tts_voice_clone_client.send_goal_async(goal_msg)
  

def main(args=None):
    rclpy.init(args=args)
    node = GPSR_Demo()
    if len(sys.argv) < 2:
        print("Usage: python3 voice_cloning.py <text>")
        sys.exit(1)
    text = sys.argv[1]

    future = node.tts_voice_clone(
        text,
        language='en',
        speaker_wav=get_wav_files_string(folder="/voices/libtrump")
    )

    rclpy.spin_until_future_complete(node, future)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
