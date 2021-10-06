ARG from=ghcr.io/robotology/gym-ignition:codespaces-stable
FROM ${from}

ARG runtime_user=root
USER $runtime_user

# Install the PyPI package in a virtualenv
ARG pip_options=""
RUN pip install ${pip_options} gym-ignition[all] &&\
    rm -r $HOME/.cache/pip

# Clone the repository to have the examples and tests
WORKDIR /github
ARG gym_ignition_branch="master"
RUN sudo chown $runtime_user /github &&\
    git clone -b $gym_ignition_branch https://github.com/robotology/gym-ignition /github
