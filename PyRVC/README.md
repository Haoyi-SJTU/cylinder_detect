# Installing PyRVC

## The Easy Way

The easiest way to install RVC Python SDK is simply to type the
following command from the command line:

```bash
cd ..
pip3 install ./PyRVC
```

That's it! You're ready to use PyRVC. Enjoy!
You can find python examples in `RVC/Examples/Python` directory.


## Using CMake

If you want to form PyRVC.so by using CMake, you can type the following commonds.

```bash
mkdir Build
cd Build
cmake ..
make -j
```

Than you can find PyRVC.so in `Build/` directory.
