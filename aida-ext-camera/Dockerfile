FROM python:3.12-slim-bookworm
WORKDIR /app
COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt
COPY identify_board.py ./
ENTRYPOINT ["python", "/app/identify_board.py"]
