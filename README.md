# Anatase Kernel patchwork

Welcome to the Anatase kernel patchwork repository. Here, you can find the patch series that is currently used in Anatase, in addition to a tagged history of all the previous series.

## Generating srpm
To generate an srpm from this repository, use one of the anatase-* branches, then run:
```bash
dist=.fc44
relver=1
make -C redhat dist-srpm -j $(expr $(nproc) - 2) \
        DIST=$dist DISTLOCALVERSION=.anatase BUILD=$relver
```

## Contributing

If you believe a patch is missing or a patch should be included, please open an issue with the patch or lore link in the [kernel-anatase](https://github.com/anatase-org/kernel-anatase) repository.

> [!WARNING]
> Do not open Pull Requests or issues in this repository. They will be closed.