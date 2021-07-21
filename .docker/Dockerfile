ARG from=diegoferigo/gym-ignition:pypi-master
FROM ${from}

# Install the PyPI package in a virtualenv
ARG pip_options=""
RUN virtualenv -p $(which python3) ${VIRTUAL_ENV} &&\
    python -m pip install --upgrade pip &&\
    pip install ${pip_options} gym-ignition &&\
    rm -r $HOME/.cache/pip

# Clone the repository
WORKDIR /github
ARG branch="master"
RUN git clone -b ${branch} https://github.com/robotology/gym-ignition /github

# Reset the entrypoint
ENTRYPOINT [""]

CMD ["bash"]
