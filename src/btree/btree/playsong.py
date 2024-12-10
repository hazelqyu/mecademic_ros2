import pygame

class Song:
    """Class representing a single song with independent control."""
    def __init__(self, file_path):
        self.file_path = file_path
        self.sound = pygame.mixer.Sound(file_path)
        self.channel = None
        self.is_paused = False  # Track whether the song is paused

    def play(self):
        """Play the song."""
        self.channel = self.sound.play()
        self.is_paused = False  # Reset pause state when starting playback

    def pause(self):
        """Pause the song."""
        if self.channel and not self.is_paused:
            self.channel.pause()
            self.is_paused = True

    def unpause(self):
        """Unpause the song."""
        if self.channel and self.is_paused:
            self.channel.unpause()
            self.is_paused = False

    def stop(self):
        """Stop the song."""
        if self.channel:
            self.channel.stop()
            self.is_paused = False  # Reset pause state when stopping playback

    def is_playing(self):
        """
        Check if the song is currently playing.
        
        Returns:
            bool: True if the song is playing, False otherwise.
        """
        return self.channel is not None and self.channel.get_busy() and not self.is_paused

    def is_paused_state(self):
        """
        Check if the song is paused.

        Returns:
            bool: True if the song is paused, False otherwise.
        """
        return self.is_paused



class MusicPlayer:
    _instance = None  # Class variable to hold the single instance

    def __new__(cls, *args, **kwargs):
        """Override __new__ to ensure only one instance is created."""
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance
    
    
    def __init__(self):
        """Initialize the MusicPlayer (runs only once)."""
        if not hasattr(self, 'initialized'):  # Ensure __init__ only runs once
            pygame.mixer.init()
            self.songs = {  # A dictionary to map song names to Song objects
                1: Song("/home/andrek/ros2_ws/songs/Piranha Plants Lullaby.mp3"),
                2: Song("/home/andrek/ros2_ws/songs/Piranha_song.mp3"),
            }
            self.initialized = True  # Mark as initialized

    def play_song(self, song_number):
        """Play a specific song."""
        if song_number in self.songs:
            self.songs[song_number].play()
            print(f"Playing song {song_number}")
        else:
            print("Invalid song number.")

    def pause_song(self, song_number):
        """Pause a specific song."""
        if song_number in self.songs:
            self.songs[song_number].pause()
            print(f"Paused song {song_number}")
        else:
            print("Invalid song number.")

    def unpause_song(self, song_number):
        """Unpause a specific song."""
        if song_number in self.songs:
            self.songs[song_number].unpause()
            print(f"Unpaused song {song_number}")
        else:
            print("Invalid song number.")

    def stop_song(self, song_number):
        """Stop a specific song."""
        if song_number in self.songs:
            self.songs[song_number].stop()
            print(f"Stopped song {song_number}")
        else:
            print("Invalid song number.")

    def is_playing(self, song_number):
        """Check if a specific song is playing."""
        if song_number in self.songs:
            return self.songs[song_number].is_playing()
        print("Invalid song number.")
        return False
