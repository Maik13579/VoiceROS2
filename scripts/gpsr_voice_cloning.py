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

    def send_tts_goal_sync(self, text, language='', speaker_wave='', wait_before_speaking=0.0):
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
    
    def send_tts_voice_clone_goal_sync(self, text, language='', speaker_wave='', wait_before_speaking=0.0):
        goal_msg = TTSAction.Goal()
        goal_msg.text = text
        goal_msg.language = language
        goal_msg.speaker_wave = speaker_wave
        goal_msg.wait_before_speaking = wait_before_speaking

        self.get_logger().info('Sending TTS goal...')
        self.tts_voice_clone_client.wait_for_server()
        send_goal_future = self.tts_voice_clone_client.send_goal_async(goal_msg)
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

    def send_speech_goal_sync(self, continuous=False, publish_partial=False, wave_file=''):
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

    def send_command_speech_goal_sync(self, continuous=False, publish_partial=False, wave_file=''):
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

    command_wave_path = "/scripts/command.wav"

    while rclpy.ok():

        # Ask for command
        tts_result = node.send_tts_goal_sync(
            text="Please tell me the command",
            language="",
            speaker_wave="",
            wait_before_speaking=0.0
        )

        # Listen for command
        command_speech_result = node.send_command_speech_goal_sync(
            continuous=False,
            publish_partial=False,
            wave_file=command_wave_path
        )

        # If not understood, continue
        if command_speech_result is None or len(command_speech_result.final_result.text) < 10:
            # Tell that you clouldn't understand the command
            tts_result = node.send_tts_goal_sync(
                text="I couldn't understand your command",
                language="",
                speaker_wave="",
                wait_before_speaking=0.0
            )
            continue

        understood = False
        question = f"Did you say. {command_speech_result.final_result.text}?"
        while rclpy.ok() and not understood:

            # Repeat command and ask for correctnes
            tts_result = node.send_tts_voice_clone_goal_sync(
                text=question,
                language="de",
                speaker_wave=command_wave_path,
                wait_before_speaking=0.0
            )

            # Set grammar to only understand yes or no
            grammar_response = node.call_set_grammar_sync(grammar_list=["yes", "no"])

            # Listen
            speech_result = node.send_speech_goal_sync(
                continuous=False,
                publish_partial=False,
                wave_file=""
            )

            # If not understood, continue
            if speech_result is None or len(speech_result.final_result.text) < 2:
                # Tell that you clouldn't it
                tts_result = node.send_tts_voice_clone_goal_sync(
                    text="I couldn't understand you",
                    language="en",
                    speaker_wave=command_wave_path,
                    wait_before_speaking=0.0
                )
                continue
            # if understood leave inner loop
            understood = True
        
        # if yes in result leave outer loop
        if "yes" in speech_result.final_result.text:
            break
        # lets try again
        tts_result = node.send_tts_voice_clone_goal_sync(
            text="okay let's try again",
            language="en",
            speaker_wave=command_wave_path,
            wait_before_speaking=0.0
        )


    # Tell that you will do the command
    tts_result = node.send_tts_voice_clone_goal_sync(
        text="Okay I will do it",
        language="en",
        speaker_wave=command_wave_path,
        wait_before_speaking=0.0
    )

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
