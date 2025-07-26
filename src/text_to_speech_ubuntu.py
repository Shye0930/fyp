from gtts import gTTS
import os
import subprocess
import platform

def text_to_speech_and_play_ubuntu():
    """
    Prompts the user for a sentence, converts it to speech, saves it as an MP3,
    and then attempts to play it using system's default media player for Ubuntu 20.04.
    """
    print("--- Text-to-Speech Generator & Player for Ubuntu 20.04 ---")
    user_sentence = input("Type the sentence you want to play: ")

    if not user_sentence.strip():
        print("No sentence entered. Exiting.")
        return

    audio_filename = "typed_sentence_audio.mp3"

    try:
        # Create a gTTS object
        tts = gTTS(text=user_sentence, lang='en', slow=False)

        # Save the audio file
        tts.save(audio_filename)
        print(f"Sentence saved as '{audio_filename}'")

        # --- Automatic Playback Part for Ubuntu ---
        print("Attempting to play the audio file using xdg-open...")

        # xdg-open is the preferred way on Linux desktop environments like Ubuntu
        try:
            subprocess.run(["xdg-open", audio_filename], check=True)
            print("Playback command issued via xdg-open.")
        except subprocess.CalledProcessError as e:
            print(f"xdg-open failed (Error: {e}). Trying common audio players...")
            # Fallback to common command-line players if xdg-open doesn't work
            try:
                # mpv is a good command-line player, often lightweight
                subprocess.run(["mpv", audio_filename], check=True)
                print("Playback command issued via mpv.")
            except FileNotFoundError:
                try:
                    # vlc can also be used from the command line
                    subprocess.run(["vlc", "--play-and-exit", audio_filename], check=True)
                    print("Playback command issued via vlc.")
                except FileNotFoundError:
                    print("No suitable audio player (xdg-open, mpv, vlc) found or configured.")
                    print("Please ensure one of these is installed and in your PATH.")
        except FileNotFoundError:
             print("xdg-open command not found. Please ensure 'xdg-utils' is installed.")
             print("You can install it with: sudo apt install xdg-utils")

        print("If your Bluetooth speaker is connected and set as the default output device, it should play there.")

    except Exception as e:
        print(f"An error occurred: {e}")
        print("Please ensure you have an active internet connection for gTTS.")

if __name__ == "__main__":
    text_to_speech_and_play_ubuntu()