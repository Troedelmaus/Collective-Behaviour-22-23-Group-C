# Collective-Behaviour-22-23-Group-C
Repository for group work in Collective Behaviour 22/23


We aim to quantitively evaluate and compare different psychological crowd modelling algorithms through the systematic exploration of state of the art Crowd Simulation models. 
Our first fuzzy logic-based model, replicating OCEAN personality crowds, published in 2020 by https://www.mdpi.com/2073-8994/9/10/239. The Model we compare it with is based upon the BioCrowds algorithm and adds a Personality layer https://repositorio.pucrs.br/dspace/bitstream/10923/14148/2/Simulating_Crowds_with_OCEAN_Personality_Traits.pdf.
Each model will be analysed on both a conceptual level and a practical level. All models will perform the same tasks and be evaluated on similar metrics, thus creating both a systematic review and an quantitative result comparison.
All development and experimentation are conducted using Vadere. Vadere is an open-
source framework for the simulation of pedestrian and crowd dynamics. Its GUI pro-
vides a toolset to create topographies, alter hyperparameters and output measure-
ments of its simulations. Quantitatively, Vadere can measure the overlaps of the pedes-
trians, their wait times and other values. The simulation setups and validation metrics
are described below in Simulation and Validation.
The reviewed models are implemented inside Vadere. Its core is a simula-
tion loop that calculates the movement of several pedestrians depending on the given
targets, topography, and a locomotion model. Vadere integrates several locomotion
models, which typically consist of a module that extends upon Vadere’s default pedes-
trian object definition and a module that moves the pedestrians called the "steering
model".

---

![](https://github.com/Troedelmaus/Collective-Behaviour-22-23-Group-C/blob/main/simulationRun.gif)

---

The following is the documentation of Vadere from https://gitlab.lrz.de/vadere/vadere:

---

![Vadere Logo](vadere.png "Vadere Logo")

---

# Vadere

Vadere is an open source framework for the simulation of microscopic pedestrian and crowd dynamics. Vadere provides generic model classes and visualisation and data analysis tools for two-dimensional systems. A series of pedestrian locomotion models are already implemented in the framework that are also the basis for scientific publications. Namely the optimal steps model, the gradient navigation model, and the social force model are available for simulation studies. In addition to pedestrians, other systems including cars and granular flow can be introduced into the framework.

This software runs on Windows, OS X and Linux.

Vadere has been developed by [Prof. Dr. Gerta Köster's](http://www.cs.hm.edu/die_fakultaet/ansprechpartner/professoren/koester/index.de.html)
research group at the [Munich University of Applied Sciences](https://www.hm.edu/) at the
[department for Computer Science and Mathematics](http://cs.hm.edu/).
However, we welcome contributions from external sources. 

The Vadere framework includes a mesh generator for unstructured high-quality 2D meshes called **EikMesh** which is described [here](https://gitlab.lrz.de/vadere/vadere/wikis/eikmesh/Overview).

If you are using Vadere for your publication, please cite: http://dx.doi.org/10.17815/CD.2019.21. 

### Vadere Units 

All measurements in Vadere are in SI units. That means, meters for the positions in the GUI and Topography, and meter/second for speeds. 

## Contact

If you have questions about Vadere, have found a software bug or have a suggestion for improvements, please feel free to either

* [open an issue](https://gitlab.lrz.de/vadere/vadere/issues) (Note: you require an LRZ account), or
* send an e-mail with your enquiry to our mailing list [vadere@lists.lrz.de](vadere@lists.lrz.de)


## Pipeline Status

| Branch  | Pipeline Status  | Coverage |
|:--------|:----------------:|:--------:| 
| master  | [![pipeline status (master)](https://gitlab.lrz.de/vadere/vadere/badges/master/pipeline.svg)](https://gitlab.lrz.de/vadere/vadere/commits/master) | [![coverage report (master)](https://gitlab.lrz.de/vadere/vadere/badges/master/coverage.svg)](https://gitlab.lrz.de/vadere/vadere/commits/master) |
| stable | [![pipeline status (stable)](https://gitlab.lrz.de/vadere/vadere/badges/stable/pipeline.svg)](https://gitlab.lrz.de/vadere/vadere/commits/stable) | [![coverage report (stable)](https://gitlab.lrz.de/vadere/vadere/badges/stable/coverage.svg)](https://gitlab.lrz.de/vadere/vadere/commits/stable) |

## Installation

### Dependencies

* Java 11 (OpenJDK recommended -> see [here](http://www.vadere.org/getting-started/) for more information) (not tested for > 11)
* OpenCL (optional but recommended -> see the [install instructions](https://gitlab.lrz.de/vadere/vadere/tree/master/Documentation/installation/OpenCL-Installation.md) for details)

### Pre-Built Releases

Latest build of master:
* [Windows](http://www.vadere.org/builds/master/vadere.master.windows.zip)
* [Linux](http://www.vadere.org/builds/master/vadere.master.linux.zip)

Stable releases and selected branch-builds:
* [https://www.vadere.org/download/](https://www.vadere.org/download/)

The ZIP file contains:
* **README.md** - this README file. 
* **vadere-gui.jar** - provides the GUI version of Vadere.
* **vadere-console.jar** - provides the command line version of Vadere and allows easy integration into other applications.
* **Scenarios** - contains test scenarios for pedestrian locomotion models. Note: The tests are also useful for a "getting started" (see below "Run Built-In Examples" for details).

### Getting Started

Either run the Vadere simulator by

- Double-clicking `vadere-gui.jar` (after installing [Java](http://www.vadere.org/getting-started/))
- Entering `path/to/openjdk/java -jar vadere-gui.jar`

### Scenario Files

<!---
@author Aleksandar Ivanov(ivanov0@hm.edu)
-->

You can find some example scenarios under [Scenarios/Demos](./Scenarios/Demos)

For more info see [Scenario File Specification](./Documentation/scenario/scenario-file-specification.md)

### Run Built-In Examples

With the following steps, you can run a simulation with one of the built-in examples from [Scenarios](Scenarios):

- Start Vadere 
- Click *Project* > *Open* 
- Choose `vadere.project` of one of the test projects, e.g. [TestOSM](https://gitlab.lrz.de/vadere/vadere/tree/master/Scenarios/ModelTests/TestOSM) and click *open*
- Select the scenario on the left and press *run selected scenario*

## Build from Source

### Dependencies

* Java 11 or above (OpenJDK recommended)
* Maven 3.0
* Git
* OpenCL (optional but recommended)

**Note:** Please, ensure that the Git executable can be found in the `PATH` variable of your operating system.

### Build Instructions

1. git clone https://gitlab.lrz.de/vadere/vadere.git
2. cd vadere
3. mvn clean
4. mvn -Dmaven.test.skip=true package

## Changelog

See [CHANGELOG.md](https://gitlab.lrz.de/vadere/vadere/blob/master/CHANGELOG.md) for a list of changes.

## JavaDoc

- [state](http://www.vadere.org/javadoc/state/index.html)

## Contribution

See [CONTRIBUTING.md](https://gitlab.lrz.de/vadere/vadere/blob/master/CONTRIBUTING.md) for how to set up the development environment and the coding guidelines.

## License

This software is licensed under the GNU Lesser General Public License ([LGPL](https://gitlab.lrz.de/vadere/vadere/blob/master/LICENSE)).

For more information: http://www.gnu.org/licenses/lgpl.html

