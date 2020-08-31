ARG from=diegoferigo/gym-ignition:pypi-master
FROM ${from}

# Install the PyPI package in a virtualenv
ARG pypi_package="gym-ignition"
RUN virtualenv -p $(which python3) ${VIRTUAL_ENV} &&\
    pip install --pre ${pypi_package} &&\
    rm -r $HOME/.cache/pip

# Clone the repository
WORKDIR /github
ARG branch="master"
RUN git clone -b ${branch} https://github.com/robotology/gym-ignition /github

CMD ["bash"]
ENTRYPOINT [""]
