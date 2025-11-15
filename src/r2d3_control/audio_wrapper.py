#!/usr/bin/env python3
"""
Audio playback wrapper for R2D3 emotional states.
Plays MP3 files from resources/<emotion>/ folders using pygame or aplay.
"""
import os
import random
import subprocess

class AudioWrapper:
    """Wrapper for playing emotion-based audio files."""
    
    def __init__(self, resources_base="/home/pi/workspace/r2d3/src/resources"):
        """
        Initialize audio wrapper.
        
        Args:
            resources_base: Path to resources folder containing emotion subfolders
        """
        self.resources_base = resources_base
        self.valid_emotions = [
            'Annoyed', 'Chat', 'Fun', 'Happy', 'Ooh', 
            'Question', 'Scared', 'Scream', 'Yell'
        ]
        self.current_process = None
    
    def get_audio_files(self, emotion):
        """
        Get list of MP3 files for given emotion.
        
        Args:
            emotion: Emotion folder name
            
        Returns:
            List of MP3 file paths, or empty list if emotion not found
        """
        emotion_dir = os.path.join(self.resources_base, emotion)
        if not os.path.isdir(emotion_dir):
            return []
        
        mp3_files = [
            os.path.join(emotion_dir, f) 
            for f in os.listdir(emotion_dir) 
            if f.endswith('.mp3')
        ]
        return sorted(mp3_files)
    
    def play_audio(self, emotion, random_file=True):
        """
        Play a random (or first) MP3 file from emotion folder.
        
        Args:
            emotion: Emotion folder name
            random_file: If True, pick random file; if False, play first
            
        Returns:
            Tuple of (success: bool, message: str)
        """
        # Validate emotion
        if emotion not in self.valid_emotions:
            return False, f"Invalid emotion '{emotion}'. Valid options: {', '.join(self.valid_emotions)}"
        
        # Get audio files
        audio_files = self.get_audio_files(emotion)
        if not audio_files:
            return False, f"No MP3 files found in {emotion} folder"
        
        # Select file
        audio_file = random.choice(audio_files) if random_file else audio_files[0]
        
        try:
            # Stop any currently playing audio
            self.stop_audio()
            
            # Try to play using aplay (standard on Raspberry Pi)
            # Falls back to other methods if aplay not available
            self.current_process = subprocess.Popen(
                ['play', audio_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            
            return True, f"Playing {os.path.basename(audio_file)} from {emotion}"
        except FileNotFoundError:
            # aplay not found, try alternative: mpg123
            try:
                self.current_process = subprocess.Popen(
                    ['mpg123', audio_file],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
                return True, f"Playing {os.path.basename(audio_file)} from {emotion} (mpg123)"
            except FileNotFoundError:
                return False, "No audio player found (aplay or mpg123 required)"
        except Exception as e:
            return False, f"Error playing audio: {str(e)}"
    
    def stop_audio(self):
        """Stop currently playing audio."""
        if self.current_process and self.current_process.poll() is None:
            self.current_process.terminate()
            try:
                self.current_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.current_process.kill()
            self.current_process = None
    
    def is_playing(self):
        """Check if audio is currently playing."""
        if self.current_process is None:
            return False
        return self.current_process.poll() is None


if __name__ == '__main__':
    # Simple test
    aw = AudioWrapper()
    print("Valid emotions:", aw.valid_emotions)
    print("Testing audio files for Happy:")
    files = aw.get_audio_files('Happy')
    print(f"  Found {len(files)} files")
    if files:
        print(f"  First file: {files[0]}")
