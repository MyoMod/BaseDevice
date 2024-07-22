# MyoMod BaseDevice

Welcome and thank you for wanting to contribute to the MyoMod platform! 

This repository provides a comprehensive template to help you create custom devices for the MyoMod Environment based on the RP2040 microncontroller by the Raspberry Foundation. MyoMod is a modular and extensible environment designed for developing and integrating muscle and movement-based interfaces primarily for the use in prosthesis.

This template will guide you through the process of creating a new device module that can be seamlessly integrated into the MyoMod ecosystem.

## Introduction

The MyoMod BaseDevice Template provides a standard framework for developing new device modules compatible with the MyoMod Environment. This template includes example code, configuration files, and a vscode dev-container to help you get started quickly and easily.

## Getting Started
### Prerequisites

Before you begin, ensure you have the following prerequisites installed:

* docker (don't use the snap package, it's not supported by vscode!)
  - Please also follow the [Post-Install steps](https://docs.docker.com/engine/install/linux-postinstall/)
* vscode
  - Install the [dev-container extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

### Installation

Clone this repository and open it in vscode. You should be prompted to open it in a dev-container - do so and you are good to go!

### Folder Structure
The template follows a structured format to organize your device module efficiently:
```
BaseDevice/
│
├── .devcontainer/ <--- these files are used to generate the docker dev-container
|
├── .vscode/ <--- these files configure vscode and the debuggers
|
├── ozone_projects/ <--- these are project files for the ozone debugger by SEGGER
│
├── myomod_interface/ <--- here lifes all the general MyoMod interface stuff
│
├── source/ <--- put your code here
│   ├── CMakeLists.txt  
│   ├── BaseDevice.cpp  <--- this contains your main function, name it whatever you want
│   └── specificRegisters.h <--- This defines your interface to the MyoMod Environment
│
├── .gitignore
├── ...
└── README.md
```