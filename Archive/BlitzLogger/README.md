# BlitzLogger
BlitzLogger creates a textfile that must be opened in NotePad++ and 
can be used to debug and take logs of what our robot is doing

There are 5 LogLevels
```
    Error    - 0
    Warning  - 1
    Info     - 2
    Debug    - 3
    Trace    - 4
```

When you start, make the constructor for the BlitzLogger item (you must
include the logLevel). When you call the individual logging functions
they will check to see if the log level you set is greater than or
equal to the log level for that funcion. As an example if you set log
level to 2, the Error, Warning, and Info functions will output to the file,
while the Debug and Trace functions will not.
```cpp
    BlitzLogger(/* Put LogLevel Here */)
```
To use BlitzLogger at the beginning of the program, you need to use the
init command
```cpp
    BlitzLogger.init();
```
After that, when you call one of the log functions

```cpp
BlitzLogger.Error();
BlitzLogger.Warning();
BlitzLogger.Info();
BlitzLogger.Debug();
BlitzLogger.Trace();
```

You need to pass it a String stating what stage of the game you are in
and and a string stating what the message you want to write is.

**Example:**
```cpp
    BlitzLogger.Error("Tele-OP", "Network Tables Disconnected");
```
After the while loop in teleop you need to close the BlizLogger
```cpp
    BlitzLogger.close();
```

Made by Sam Ferguson in 2018, archived in 2021.