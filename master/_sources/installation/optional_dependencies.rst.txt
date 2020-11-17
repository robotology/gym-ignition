.. _installation_optional_dependencies:

Optional Dependencies
=====================

.. _installation_optional_dependencies_idyntree:

iDynTree
********

**gym-ignition** provides helper classes to manipulate the kinematics and the dynamics of rigid-bodies.
Among the many existing solutions, we selected `iDynTree <https://github.com/robotology/idyntree/>`_.

Follow the `official installation instructions <https://github.com/robotology/idyntree/#installation>`__ and make sure that you also enable and install the `Python bindings <https://github.com/robotology/idyntree/#bindings>`__.

You can verify that the installation succeeded and your system is properly configured if you can ``import iDynTree`` in a Python interpreter.

.. tip::
    If you want to learn more about ``iDynTree``, the two classes we mainly use are ``iDynTree::KinDynComputations`` (`docs <https://robotology.github.io/idyntree/master/classiDynTree_1_1KinDynComputations.html>`__) and ``iDynTree::InverseKinematics`` (`docs <https://robotology.github.io/idyntree/master/classiDynTree_1_1InverseKinematics.html>`__).

    The theory and notation behind the library is summarized in `Multibody dynamics notation <https://pure.tue.nl/ws/portalfiles/portal/139293126/A_Multibody_Dynamics_Notation_Revision_2_.pdf>`_.