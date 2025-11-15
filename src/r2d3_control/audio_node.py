#!/usr/bin/env python3
"""
Audio service node for R2D3.
Provides ROS service to play emotion-based audio files.
"""
import rospy
from r2d3_control.srv import PlayAudio, PlayAudioResponse
from r2d3_control import audio_wrapper


def main():
    rospy.init_node('audio_node')
    aw = audio_wrapper.AudioWrapper()
    
    def play_audio_cb(req):
        """Service callback for playing audio by emotion."""
        success, message = aw.play_audio(req.emotion, random_file=True)
        rospy.loginfo(f"Audio service: {message}")
        return PlayAudioResponse(success=success, message=message)
    
    rospy.Service('audio/play_emotion', PlayAudio, play_audio_cb)
    rospy.loginfo("audio_node started. Service: /audio/play_emotion")
    
    rospy.spin()


if __name__ == '__main__':
    main()
