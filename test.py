import pdfplumber as pp
from gtts import gTTS

pdf_text = ''
with pp.open('/home/qin/Downloads/TIEPaper/etal_Submitted.pdf') as pdf:
    for page in pdf.pages:
        pdf_text += page.extract_text()

tts = gTTS(text=pdf_text, lang='en')
tts.save('audio_book.mp3')