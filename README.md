# FRC-2019

![](https://img.shields.io/github/contributors/SkylineSpartabots/FRC-2019.svg?style=flat-square)
![](https://img.shields.io/github/languages/code-size/SkylineSpartabots/FRC-2019.svg?style=flat-square)
![](https://img.shields.io/github/license/SkylineSpartabots/FRC-2019.svg?style=flat-square)

The Spartabots' FRC robot code for the DESTINATION: DEEP SPACE game!

## Setup

1. Install the NI Update Suite and the WPILib Development Tools (+VS Code) ([Instructions][installation-instructions])
2. Install the libraries we use (CTRE Phoenix and NavX).
3. Make a new WPILib Java project and clone this repository

### Libraries
We use two 3rd party libraries:
- [CTRE Phoenix](http://www.ctr-electronics.com/control-system/hro.html#product_tabs_technical_resources) (Currently using v5.12.0)
- [NavX](https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/) (Currently using v3.1.344)

[Instructions](https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/682619-3rd-party-libraries#adding-an-offline-installed-library) for adding an offline library to VS Code.

## Contributing
- Commit your changes to a feature branch, not to master, _especially if it is untested_
- Write [good][good-commit-message] commit messages
	- Written in sentence case
	- Should complete the sentence: "If applied, this commit will _[Your commit message]_."
- Make sure each of your methods has a [Javadoc comment][javadoc-comment] and you comment liberally
- Maintain a consistent code style

## Useful links
- [Game animation](https://www.youtube.com/watch?v=Mew6G_og-PI)
- [Game manual](https://firstfrc.blob.core.windows.net/frc2019/Manual/2019FRCGameSeasonManual.pdf)
- [Screensteps documentation](https://wpilib.screenstepslive.com/s/currentCS/m/java)
- [WPILib API documentation](http://first.wpi.edu/FRC/roborio/release/docs/java/)

[installation-instructions]: https://wpilib.screenstepslive.com/s/currentCS/m/java/l/1027504-installing-the-frc-update-suite-all-languages
[good-commit-message]: https://juffalow.com/other/write-good-git-commit-message
[javadoc-comment]: https://en.wikipedia.org/wiki/Javadoc#Structure_of_a_Javadoc_comment