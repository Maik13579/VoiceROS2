#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from coqui_tts_ros2_interfaces.action import TTS as TTSAction
from vosk_ros2_interfaces.action import SpeechDetection
from vosk_ros2_interfaces.srv import SetGrammar

class GPSR_Demo(Node):
    def __init__(self):
        super().__init__('gpsr_demo')
        # Action clients for TTS and Speech Detection
        self.tts_client = ActionClient(self, TTSAction, '/tts/tts')

        self.speech_client = ActionClient(self, SpeechDetection, '/vosk/speech_detection')
        self.command_speech_client = ActionClient(self, SpeechDetection, '/vosk_gpsr/speech_detection')
        # Service client for set_grammar
        self.grammar_client = self.create_client(SetGrammar, '/vosk/set_grammar')

    def tts(self, text, language='', speaker_wave='', wait_before_speaking=0.0):
        goal_msg = TTSAction.Goal()
        goal_msg.text = text
        goal_msg.language = language
        goal_msg.speaker_wave = speaker_wave
        goal_msg.wait_before_speaking = wait_before_speaking

        self.get_logger().info('Sending TTS goal...')
        self.tts_client.wait_for_server()
        send_goal_future = self.tts_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('TTS goal rejected.')
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'TTS result: success={result.success}, message={result.message}')
        return result
    
    def listen(self, continuous=False, publish_partial=False, wave_file=''):
        goal_msg = SpeechDetection.Goal()
        goal_msg.continuous = continuous
        goal_msg.publish_partial = publish_partial
        goal_msg.wave_file = wave_file

        self.get_logger().info('Sending Speech Detection goal...')
        self.speech_client.wait_for_server()
        send_goal_future = self.speech_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Speech Detection goal rejected.')
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'Speech Detection result: {result.final_result.text}')
        return result

    def listen_command(self, continuous=False, publish_partial=False, wave_file=''):
        goal_msg = SpeechDetection.Goal()
        goal_msg.continuous = continuous
        goal_msg.publish_partial = publish_partial
        goal_msg.wave_file = wave_file

        self.get_logger().info('Sending Command Speech Detection goal...')
        self.command_speech_client.wait_for_server()
        send_goal_future = self.command_speech_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Command Speech Detection goal rejected.')
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f'Command Speech Detection result: {result.final_result.text}')
        return result

    def set_grammar(self, grammar_list):
        self.get_logger().info('Calling set_grammar service synchronously...')
        if not self.grammar_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Service /set_grammar not available.')
            return None
        request = SetGrammar.Request()
        request.grammar = grammar_list  # List of strings
        future = self.grammar_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f'SetGrammar response: success={response.success}, message={response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GPSR_Demo()

    while rclpy.ok():
        node.tts(text="Please tell me the command")

        result = node.listen_command()
        if result is None or len(result.final_result.text) < 10:
            node.tts(text="I couldn't understand your command")
            continue

        understood = False
        while rclpy.ok() and not understood:
            node.tts(text=f"Did you say. {result.final_result.text}?")

            node.set_grammar(grammar_list=["yes", "no"])
            result = node.listen()

            if result is None or len(result.final_result.text) < 2:
                node.tts(text="I couldn't understand you")
                continue
            understood = True
        if "yes" in result.final_result.text:
            break
        node.tts(text="okay let's try again")
    node.tts(text="Okay I will do it")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
