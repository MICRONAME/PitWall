# PitWall 0.3.0

Готовы физика машины, замкнутая трасса и проверка кругов. Графического окна пока нет.
Начни с [TRACK_STAGE_RU.md](TRACK_STAGE_RU.md): там команды запуска и описание этапа.
История этапа физики остаётся в NEXT_STAGE_RU.md.

```powershell
$env:Path = 'C:\msys64\ucrt64\bin;' + $env:Path
cmake --preset windows-mingw
cmake --build --preset windows-mingw --parallel 2
ctest --preset windows-mingw
.\build\windows-mingw\apps\simulator\pitwall_simulator.exe --track-demo
```

Все команды выполняются из папки с CMakeLists.txt. На MinGW и MSVC прошли все 58 тестов.
Обычная сборка не требует внешних библиотек. GoogleTest/vcpkg подготовлены отдельно
и пока не проверены. Следующий этап — графика и управление машиной с клавиатуры.