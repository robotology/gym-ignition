# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import numpy as np
from lxml import etree
from pathlib import Path
from enum import auto, Enum
from typing import Dict, List, NamedTuple, Union


class Distribution(Enum):
    Uniform = auto()
    Gaussian = auto()


class Method(Enum):
    Absolute = auto()
    Additive = auto()
    Coefficient = auto()


class RandomizationData(NamedTuple):
    xpath: str
    distribution: str
    parameters: "DistributionParameters"
    method: Method
    ignore_zeros: bool = False
    force_positive: bool = False
    element: etree.Element = None


class GaussianParams(NamedTuple):
    variance: float
    mean: float = None


class UniformParams(NamedTuple):
    low: float
    high: float


DistributionParameters = Union[UniformParams, GaussianParams]


class RandomizationDataBuilder:

    def __init__(self, randomizer: "SDFRandomizer"):

        self.storage: Dict = {}
        self.randomizer = randomizer

    def at_xpath(self, xpath: str) -> "RandomizationDataBuilder":
        self.storage["xpath"] = xpath
        return self

    def sampled_from(self,
                     distribution: Distribution,
                     parameters: DistributionParameters) -> "RandomizationDataBuilder":

        self.storage["distribution"] = distribution
        self.storage["parameters"] = parameters

        if self.storage["distribution"] is Distribution.Gaussian and \
            not isinstance(parameters, GaussianParams):
            raise ValueError("Wrong parameters type")

        if self.storage["distribution"] is Distribution.Uniform and \
            not isinstance(parameters, UniformParams):
            raise ValueError("Wrong parameters type")

        return self

    def method(self, method: Method) -> "RandomizationDataBuilder":

        self.storage["method"] = method
        return self

    def ignore_zeros(self, ignore_zeros: bool) -> "RandomizationDataBuilder":
        self.storage["ignore_zeros"] = ignore_zeros
        return self

    def force_positive(self, force_positive: bool = True) -> "RandomizationDataBuilder":
        self.storage["force_positive"] = force_positive
        return self

    def add(self) -> None:

        data = RandomizationData(**self.storage)

        if len(self.randomizer.find_xpath(data.xpath)) == 0:
            raise RuntimeError(f"Failed to find element matching XPath '{data.xpath}'")

        self.randomizer.insert(randomization_data=data)


class SDFRandomizer:

    def __init__(self, sdf_model: str):

        self._sdf_file = sdf_model

        if not Path(self._sdf_file).is_file():
            raise ValueError(f"File '{sdf_model}' does not exist")

        # Initialize the root
        tree = self._get_tree_from_file(self._sdf_file)
        self._root: etree.Element = tree.getroot()

        # List of randomizations
        self._randomizations: List[RandomizationData] = []

        # List of default values used with Method.Coefficient
        self._default_values: Dict[etree.Element, float] = {}

        # Store an independent RNG
        self.rng = np.random.default_rng()

    def seed(self, seed: int) -> None:
        self.rng = np.random.default_rng(seed)

    def find_xpath(self, xpath: str) -> List[etree.Element]:
        return self._root.findall(xpath)

    def process_data(self) -> None:

        # Since we support multi-match XPaths, we expand all the individual matches
        expanded_randomizations = []

        for data in self._randomizations:

            # Find all the matches
            elements: List[etree.Element] = self._root.findall(path=data.xpath)

            if len(elements) == 0:
                raise RuntimeError(f"Failed to find elements from XPath '{data.xpath}'")

            for element in elements:

                if data.ignore_zeros and float(self._get_element_text(element)) == 0:
                    continue

                # Get the precise XPath to the element
                element_xpath = element.getroottree().getpath(element)

                # Get the parameters
                params = data.parameters

                if data.method in {Method.Additive, Method.Coefficient}:
                    element_text = float(self._get_element_text(element))
                    self._default_values[element] = element_text

                # Update the data
                complete_data = data._replace(
                    xpath=element_xpath, element=element, parameters=params)

                expanded_randomizations.append(complete_data)

        # Store the updated data
        self._randomizations = expanded_randomizations

    def sample(self, pretty_print=False) -> str:

        for data in self._randomizations:

            if data.distribution is Distribution.Gaussian:

                sample = self.rng.normal(loc=data.parameters.mean,
                                         scale=data.parameters.variance)

            elif data.distribution is Distribution.Uniform:

                sample = self.rng.uniform(low=data.parameters.low,
                                          high=data.parameters.high)

            else:
                raise ValueError("Distribution not recognized")

            if data.force_positive:
                sample = max(sample, 0.0)

            # Update the value
            if data.method is Method.Absolute:

                data.element.text = str(sample)

            elif data.method is Method.Additive:

                default_value = self._default_values[data.element]
                data.element.text = str(sample + default_value)

            elif data.method is Method.Coefficient:

                default_value = self._default_values[data.element]
                data.element.text = str(sample * default_value)

            else:
                raise ValueError("Method not recognized")

        return etree.tostring(self._root, pretty_print=pretty_print).decode()

    def new_randomization(self) -> RandomizationDataBuilder:
        return RandomizationDataBuilder(randomizer=self)

    def insert(self, randomization_data) -> None:
        self._randomizations.append(randomization_data)

    def get_active_randomizations(self) -> List[RandomizationData]:
        return self._randomizations

    def clean(self) -> None:

        self._randomizations = []
        self._default_values = {}

        tree = self._get_tree_from_file(self._sdf_file)
        self._root = tree.getroot()

    @staticmethod
    def _get_tree_from_file(xml_file) -> etree.ElementTree:

        parser = etree.XMLParser(remove_blank_text=True)
        tree = etree.parse(source=xml_file, parser=parser)

        return tree

    @staticmethod
    def _get_element_text(element: etree.Element) -> str:

        text = element.text

        if text is None:
            raise RuntimeError(f"The element {element.tag} does not have any content")

        return text
