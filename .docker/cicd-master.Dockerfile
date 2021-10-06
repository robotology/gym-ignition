ARG from=ghcr.io/robotology/gym-ignition:codespaces-base
FROM ${from}

ARG runtime_user=root
USER $runtime_user

# Install Ignition Gazebo
ARG ignition_codename="fortress"
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list &&\
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - &&\
    sudo apt-get update &&\
    sudo apt-get install -y --no-install-recommends ignition-$ignition_codename &&\
    sudo rm -rf /var/lib/apt/lists/*
