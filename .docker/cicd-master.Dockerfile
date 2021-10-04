ARG from=diegoferigo/gym-ignition:base
FROM ${from}

# Install ignition gazebo
ARG ignition_codename="fortress"
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" \
        > /etc/apt/sources.list.d/gazebo-stable.list &&\
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - &&\
    apt-get update &&\
    apt-get install -y --no-install-recommends ignition-${ignition_codename} &&\
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
COPY setup_virtualenv.sh /setup_virtualenv.sh
RUN chmod 755 /entrypoint.sh
RUN chmod 755 /setup_virtualenv.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
