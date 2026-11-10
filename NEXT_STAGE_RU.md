# PitWall 0.2.0: физическая модель

В симулятор добавлены ускорение, торможение, повороты, сопротивление движению,
ограничение скорости и фиксированный шаг 10 мс. Графического окна пока нет.

Из корня проекта в PowerShell:

```powershell
$env:Path = 'C:\msys64\ucrt64\bin;' + $env:Path
cmake --preset windows-mingw
cmake --build --preset windows-mingw --parallel 2
ctest --preset windows-mingw
.\build\windows-mingw\apps\simulator\pitwall_simulator.exe --demo
```

Демонстрация быстро рассчитывает 12 секунд движения: разгон, левый поворот и
торможение. В таблице выводятся время, скорость в км/ч, координаты и направление.
В конце должно быть `demo completed: steps=1200`. Ожидать 12 реальных секунд не нужно.

## Какие файлы читать

1. `libs/simulation/include/pitwall/simulation/vehicle.hpp`: параметры, состояние,
   нормированные управляющие сигналы и интерфейс модели.
2. `libs/simulation/src/vehicle.cpp`: силы, скорость, ограничение поворота и траектория.
3. `libs/simulation/include/pitwall/simulation/fixed_step.hpp` и `src/fixed_step.cpp`:
   накопление времени кадров, шаг 10 мс, защита от перегрузки и телеметрия.
4. `apps/simulator/main.cpp`: сценарий демонстрации.
5. `tests/physics_cases.hpp`: аналитические проверки и сравнение 30/144 FPS.
6. `docs/physics.md`: формулы, ограничения и принятые решения.

Модель независима от графики: следующий шаг — замкнутая трасса, контрольные точки
и определение кругов, после чего подключим окно и клавиатуру.

Сообщение коммита: `feat: add fixed-step vehicle physics and headless demo`.
