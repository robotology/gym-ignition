# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from typing import List
from abc import ABC, abstractmethod


class RobotContacts(ABC):
    """
    Robot contacts interface.

    This interface provides methods to get and set contact-related quantities.
    """

    @abstractmethod
    def links_in_contact(self) -> List[str]:
        """
        Return a list of link names with active contacts.

        Returns:
            The names of the link in contact.
        """

    @abstractmethod
    def contact_data(self, contact_link_name: str):  # TODO
        """
        Return data related to a contact.

        Args:
            contact_link_name: The name of the link.

        Returns:
            A Tuple with contact data with the following information:

            # TODO ContactWrench idyntree

        """

    @abstractmethod
    def total_contact_wrench_on_link(self, contact_link_name: str) -> np.ndarray:
        """
        Return the total wrench applied to the link expressed in the link frame.

        Args:
            contact_link_name: The name of the link.

        Returns:
            The wrench applied to the link as a 6D array.
        """
