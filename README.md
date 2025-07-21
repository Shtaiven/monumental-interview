# Monumental Interview Assignment

## Installation

To bootstrap an amd64 linux system with `bash` and either `wget` or `curl`, enter the command

```bash
./bootstrap.sh
```

This will install `pixi` for the current user (which updates the local `.bashrc`) and sources
the shell again. **Bootstrapping only has to be done once** if pixi is not installed on the system.

To install all dependencies, enter the following command from the root of this project

```bash
pixi install
```

## Building

To build the application with pixi, run

```bash
pixi run build
```

## Running

To run the application, enter the command

```bash
pixi run start
```
