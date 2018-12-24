# Robot Code

### Compiling

**Windows:** gradlew build

**MacOS/Linux:** ./gradlew build


**Note:** On macOS and linux, gradlew does not have permission to run immediately. Run the following: sudo chmod +x gradlew

### Log Tools

Logging is done using the badlog library: https://github.com/dominikWin/badlog
To display the log in a graphical format, transfer the log file from the roborio and use the following program to convert it to
an html file: https://github.com/dominikWin/badlogvis. This file can then be opened in a web browser.
