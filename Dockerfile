FROM python:3.11-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
  git \
  vim \
  cmake \
  build-essential \
  default-jdk \
  wget \
  unzip \
  && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir \
  psutil networkx "numpy==1.26.4" matplotlib scipy \
  unified-planning "unified-planning[fast-downward]"

ENV JAVA_HOME=/usr/lib/jvm/default-java

WORKDIR /gradle
RUN wget -q https://services.gradle.org/distributions/gradle-8.13-bin.zip \
  && mkdir -p /opt/gradle \
  && unzip -d /opt/gradle gradle-8.13-bin.zip \
  && rm gradle-8.13-bin.zip
ENV PATH=$PATH:/opt/gradle/gradle-8.13/bin

WORKDIR /
RUN git clone https://github.com/prismmodelchecker/prism.git -b v4.10.1
RUN cd prism/prism && make && make test
RUN ln -s /prism/prism/bin/prism /usr/local/bin/prism

WORKDIR /
RUN git clone https://github.com/aibasel/downward.git -b release-24.06.1
WORKDIR /downward
RUN ./build.py
ENV PATH=/downward:$PATH

WORKDIR /
RUN git clone https://github.com/remaro-network/dlToPlanning.git
WORKDIR /dlToPlanning
RUN ./gradlew shadowJar
ENV PATH=/dlToPlanning:$PATH

WORKDIR /navigation_planta
COPY navigation_planta/ navigation_planta/
COPY scripts/ scripts/
COPY owl/ owl/
COPY pddl/ pddl/
COPY prism/ prism/
COPY data/map_camara_2020_paper/ data/map_camara_2020_paper/
COPY setup.py .
RUN pip install --no-cache-dir -e .
