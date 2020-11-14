Virtual Environment (optional)
******************************

This step is optional but highly recommended.
Visit the `virtual environments <https://docs.python.org/3.6/tutorial/venv.html>`_ documentation for more details.

.. code-block:: bash

   sudo apt install virtualenv
   virtualenv -p python3.8 $HOME/venv
   source $HOME/venv/bin/activate

Note that the activation is temporary and it is valid only in the same terminal.
Make sure to execute the next steps in a terminal where the virtual environment is active.
