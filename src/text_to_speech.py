from gtts import gTTS
import os

def text_to_speech_to_file():
    """
    Prompts the user for a sentence, converts it to speech using Google Text-to-Speech,
    and saves it as an MP3 file.
    """
    print("--- Text-to-Speech Generator ---")
    user_sentence = input("Type the sentence you want to play: ")

    if not user_sentence.strip():
        print("No sentence entered. Exiting.")
        return

    try:
        # Create a gTTS object
        tts = gTTS(text=user_sentence, lang='en', slow=False)

        # Define the filename for the audio
        audio_filename = "typed_sentence_audio.mp3"

        # Save the audio file
        tts.save(audio_filename)
        print(f"Sentence saved as '{audio_filename}'")
        print("\nNow, manually connect to your Bluetooth speaker and play this file.")

    except Exception as e:
        print(f"An error occurred: {e}")
        print("Please ensure you have an active internet connection for gTTS.")

if __name__ == "__main__":
    text_to_speech_to_file()