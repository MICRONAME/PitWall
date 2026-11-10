# PitWall 0.2.0

Это проект на C++20 с независимой физической моделью машины и консольной демонстрацией.
Графического окна пока нет. Подробные команды запуска и объяснение новых файлов
находятся в [NEXT_STAGE_RU.md](NEXT_STAGE_RU.md).

Сборка и тесты проверены на MinGW и Visual Studio 2022: 30 из 30 проверок прошли.
Внешние зависимости для обычной сборки не нужны. Дополнительный режим GoogleTest
подготовлен, но ещё не проверен.

```powershell
$env:Path = 'C:\msys64\ucrt64\bin;' + $env:Path
cmake --preset windows-mingw
cmake --build --preset windows-mingw --parallel 2
ctest --preset windows-mingw
.\build\windows-mingw\apps\simulator\pitwall_simulator.exe --demo
```

Все команды выполняются из папки PitWall, где лежит CMakeLists.txt.
Сначала читай libs/simulation/include/pitwall/simulation/vehicle.hpp, затем
libs/simulation/src/vehicle.cpp и docs/physics.md.