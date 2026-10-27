FROM ext_cam-calibration
RUN apt-get update && apt-get install -y --no-install-recommends chromium \
    && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir playwright httpx
WORKDIR /work
CMD ["python", "-m", "unittest", "discover", "-s", "tests", "-v"]
