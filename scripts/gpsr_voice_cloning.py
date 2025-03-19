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
        self.tts_voice_clone_client = ActionClient(self, TTSAction, '/tts_voice_clone/tts')

        self.speech_client = ActionClient(self, SpeechDetection, '/vosk/speech_detection')
        self.command_speech_client = ActionClient(self, SpeechDetection, '/vosk_gpsr/speech_detection')
        # Service client for set_grammar
        self.grammar_client = self.create_client(SetGrammar, '/vosk/set_grammar')

    def tts(self, text, language='', speaker_wav='', wait_before_speaking=0.0):
        goal_msg = TTSAction.Goal()
        goal_msg.text = text
        goal_msg.language = language
        goal_msg.speaker_wav = speaker_wav
        goal_msg.wait_before_speaking = wait_before_speaking

        self.get_logger().info('Sending TTS goal...')
        self.tts_client.wait_for_server()
        return self.tts_client.send_goal_async(goal_msg)
        
    
    def tts_voice_clone(self, text, language='en', speaker_wav='', wait_before_speaking=0.0):
        goal_msg = TTSAction.Goal()
        goal_msg.text = text
        goal_msg.language = language
        goal_msg.speaker_wav = speaker_wav
        goal_msg.wait_before_speaking = wait_before_speaking

        self.get_logger().info('Sending TTS goal...')
        self.tts_voice_clone_client.wait_for_server()
        self.tts_voice_clone_client.send_goal_async(goal_msg)
        

    def listen(self, continuous=False, publish_partial=False, wav_file=''):
        goal_msg = SpeechDetection.Goal()
        goal_msg.continuous = continuous
        goal_msg.publish_partial = publish_partial
        goal_msg.wav_file = wav_file

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

    def listen_command(self, continuous=False, publish_partial=False, wav_file=''):
        goal_msg = SpeechDetection.Goal()
        goal_msg.continuous = continuous
        goal_msg.publish_partial = publish_partial
        goal_msg.wav_file = wav_file

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

    def call_set_grammar_sync(self, grammar_list):
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

    command_wav_path = "/scripts/command.wav"

    while rclpy.ok():
        node.tts(text="Please tell me the command")

        result = node.listen_command(wav_file=command_wav_path)
        if result is None or len(result.final_result.text) < 10:
            node.tts(text="I couldn't understand your command")
            continue

        understood = False
        question = f"Did you say. {result.final_result.text}?"
        while rclpy.ok() and not understood:
            node.tts_voice_clone(
                text=question,
                speaker_wav=command_wav_path,
            )
            node.call_set_grammar_sync(grammar_list=["yes", "no"])
            result = node.listen()
            if result is None or len(result.final_result.text) < 2:
                node.tts_voice_clone(
                    text="I couldn't understand you",
                    speaker_wav=command_wav_path,
                )
                continue
            understood = True
        if "yes" in result.final_result.text:
            break
        future= node.tts_voice_clone(
            text="okay let's try again",
            speaker_wav=command_wav_path,
        )
        rclpy.spin_until_future_complete(node, future)
    node.tts_voice_clone(
        text="Okay I will do it",
        speaker_wav=command_wav_path,
    )

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
