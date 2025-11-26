# Build Stage
FROM ubuntu:22.04 AS builder

RUN apt-get update && apt-get install -y \
	build-essential \
	cmake \
	can-utils\
	python3 \
	python3-pip \
	&& rm -rf /var/lib/apt/lists/*
RUN pip3 install cantools

WORKDIR /app
COPY . .
RUN cmake -B build -S .
RUN cmake --build build

# Run Stage
FROM ubuntu:22.04 
RUN apt-get update && apt-get install -y \
	can-utils \
	iproute2 \
	&& rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY --from=builder /app/build/sendloop .
CMD [ "./sendloop" ]

