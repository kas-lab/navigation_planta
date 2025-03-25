FROM ubuntu:22.04

RUN apt update && apt install -y --no-install-recommends \
  git \
  vim \
  cmake \
  g++ \
  make \
  python3 \
  build-essential \
  python-is-python3 \
  python3-pip \
  openjdk-17-jdk \
  wget \
  unzip \
  && rm -rf /var/lib/apt/lists/*

RUN pip install networkx numpy==1.26.4 matplotlib scipy unified-planning unified-planning[fast-downward]

ENV JAVA_HOME=/usr/lib/jvm/java-17-openjdk-amd64

WORKDIR /gradle
RUN wget -c https://services.gradle.org/distributions/gradle-8.13-bin.zip
RUN mkdir /opt/gradle
RUN unzip -d /opt/gradle gradle-8.13-bin.zip
ENV PATH=$PATH:/opt/gradle/gradle-8.13/bin

WORKDIR /
RUN git clone https://github.com/prismmodelchecker/prism.git
RUN cd prism/prism && make && make test
RUN ln -s /prism/prism/bin/prism /usr/local/bin/prism

WORKDIR /
RUN git clone https://github.com/aibasel/downward.git
WORKDIR /downward
RUN ./build.py
ENV PATH=/downward:$PATH

WORKDIR /
RUN git clone https://github.com/remaro-network/dlToPlanning.git
WORKDIR /dlToPlanning
RUN ./gradlew shadowJar
ENV PATH=/dlToPlanning:$PATH

WORKDIR /navigation_pddl_tomasys
COPY owl/ owl/
COPY pddl/ pddl/
COPY scripts/ scripts/
COPY map_camara_2020_paper/map-p2cp3.json map_camara_2020_paper/map-p2cp3.json

WORKDIR /navigation_pddl_tomasys