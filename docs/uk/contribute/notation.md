# Термінологія

Наступні терміни, символи та декоратори використовуються в тексті та діаграмах протягом цього керівництва.

## Записка

- Жирним шрифтом позначаються вектори або матриці, а нежирним - скаляри.
- Стандартна система координат для кожної змінної - місцева система координат: $\ell{}$.
  Right [superscripts](#superscripts) represent the coordinate frame.Якщо правий верхній індекс відсутній, то припускається, що використовується стандартна система координат $\ell{}$.
  Виняток становлять матриці обертання, де нижні праві індекси вказують поточну систему координат, а верхні праві індекси - цільову систему координат.
- Змінні та нижні символи можуть використовувати однакові літери, але вони завжди мають різне значення.

## Акроніми

| Акронім      | Розшифровка                                                                                                                                                                                             |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| AOA          | Кут атаки. Also named _alpha_.                                                                                                                                          |
| AOS          | Кут бокового зсуву Also named _beta_.                                                                                                                                                   |
| FRD          | Система координат, де вісь X вказує напрямок до передньої частини транспортного засобу, вісь Y вказує напрямок праворуч, а вісь Z вказує напрямок вниз, завершуючи правило правої руки. |
| FW           | Закріплені крила (літаки).                                                                                                                                           |
| MC           | Мультикоптери.                                                                                                                                                                          |
| MPC або MCPC | Контролер позиції багатокоптера&#xA;MPC також використовується для Model Predictive Control (Модельно-прогностичне управління).                                      |
| NED          | Система координат, де вісь X вказує напрямок до справжнього Північного полюса, вісь Y вказує напрямок східної сторони, а вісь Z вказує напрямок вниз, завершуючи правило правої руки.   |
| PID          | Контролер з пропорційною, інтегральною та диференціальною діями.                                                                                                                        |

## Символи

| Змінні                                                | Опис                                                                                                                                                                                                                                      |
| ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| $x,y,z$                                               | Переміщення вздовж координатних осей x, y та z відповідно.                                                                                                                                                                |
| $\boldsymbol{\mathrm{r}}$                             | Вектор позиції: $\boldsymbol{\mathrm{r}} = [x \quad y \quad z]^T$                                                                                                     |
| $\boldsymbol{\mathrm{v}}$                             | Вектор швидкості: $\boldsymbol{\mathrm{v}} = \boldsymbol{\mathrm{\dot{r}}}$                                                                                                                                               |
| $\boldsymbol{\mathrm{a}}$                             | Прискорення vector: $\boldsymbol{\mathrm{a}} = \boldsymbol{\mathrm{\dot{v}}} = \boldsymbol{\mathrm{\ddot{r}}}$                                                                                                            |
| $\alpha$                                              | Кут атаки (AOA).                                                                                                                                                                                       |
| $b$                                                   | Розмах крила (від кінчика до кінчика).                                                                                                                                                                 |
| $S$                                                   | Площа крила                                                                                                                                                                                                                               |
| $AR$                                                  | Аспектне відношення: $AR = \frac{b^2}{S}$                                                                                                                                                                                 |
| $\beta$                                               | Кут бокового зсуву (AOS)                                                                                                                                                                                               |
| $c$                                                   | Довжина хорди крила                                                                                                                                                                                                                       |
| $\delta$                                              | Кутове відхилення аеродинамічної керуючої поверхні Позитивне відхилення породжує негативний момент.                                                                                                                       |
| $\phi,\theta,\psi$                                    | Euler angles roll (=Bank), pitch and yaw (=Heading).                                                                                                                                |
| $\Psi$                                                | Attitude vector: $\Psi = [\phi \quad \theta \quad \psi]^T$                                                                                                            |
| $X,Y,Z$                                               | Сили вздовж координатних осей x, y та z.                                                                                                                                                                                  |
| $\boldsymbol{\mathrm{F}}$                             | Вектор сили: $\boldsymbol{\mathrm{F}} = [X \quad Y \quad Z]^T$                                                                                                        |
| $D$                                                   | Сила опору                                                                                                                                                                                                                                |
| $C$                                                   | Сила бічного вітру                                                                                                                                                                                                                        |
| $L$                                                   | Сила підйому                                                                                                                                                                                                                              |
| $g$                                                   | Гравітація                                                                                                                                                                                                                                |
| $l,m,n$                                               | Моменти навколо координатних осей x, y та z.                                                                                                                                                                              |
| $\boldsymbol{\mathrm{M}}$                             | Вектор моменту: $\boldsymbol{\mathrm{M}} = [l \quad m \quad n]^T$v                                                                                                    |
| $M$                                                   | Mach number. Може бути ігноровано для масштабних літаків.                                                                                                                                                 |
| $\boldsymbol{\mathrm{q}}$                             | Векторна частина кватерніону                                                                                                                                                                                                              |
| $\boldsymbol{\mathrm{\tilde{q}}}$                     | Hamiltonian attitude quaternion (see `1` below)                                                                                                                                                                        |
| $\boldsymbol{\mathrm{R}}_\ell^b$ | Матриця повороту. Повертає вектор із системи координат $\ell{}$ в систему координат $b{}$. $\boldsymbol{\mathrm{v}}^b = \boldsymbol{\mathrm{R}}_\ell^b \boldsymbol{\mathrm{v}}^\ell$ |
| $\Lambda$                                             | Кут нахилу переднього краю                                                                                                                                                                                                                |
| $\lambda$                                             | Відношення залишкової хорди: $\lambda = \frac{c_{tip}}{c_{root}}$                                                                                                               |
| $w$                                                   | Швидкість вітру.                                                                                                                                                                                                          |
| $p,q,r$                                               | Кутові швидкості навколо тілесних осей x, y та z.                                                                                                                                                                         |
| $\boldsymbol{\omega}^b$                               | Вектор кутової швидкості в тілесній системі координат: $\boldsymbol{\omega}^b = [p \quad q \quad r]^T$                                                                |
| $\boldsymbol{\mathrm{x}}$                             | Загальний вектор стану                                                                                                                                                                                                                    |

- `1` Hamiltonian attitude quaternion. $\boldsymbol{\mathrm{\tilde{q}}} = (q_0, q_1, q_2, q_3) = (q_0, \boldsymbol{\mathrm{q}})$.<br> $\boldsymbol{\mathrm{\tilde{q}}}{}$ describes the attitude relative to the local frame $\ell{}$. To represent a vector in local frame given a vector in body frame, the following operation can be used: $\boldsymbol{\mathrm{\tilde{v}}}^\ell = \boldsymbol{\mathrm{\tilde{q}}} \, \boldsymbol{\mathrm{\tilde{v}}}^b \, \boldsymbol{\mathrm{\tilde{q}}}^_{}$ (or $\boldsymbol{\mathrm{\tilde{q}}}^{-1}{}$ instead of $\boldsymbol{\mathrm{\tilde{q}}}^_{}$ if $\boldsymbol{\mathrm{\tilde{q}}}{}$ is not unitary). $\boldsymbol{\mathrm{\tilde{v}}}{}$ represents a _quaternionized_ vector: $\boldsymbol{\mathrm{\tilde{v}}} = (0,\boldsymbol{\mathrm{v}})$

### Нижні індекси / Показники

| Нижні індекси / Показники | Опис                                                                                                  |
| ------------------------- | ----------------------------------------------------------------------------------------------------- |
| $a$                       | Aileron.                                                                              |
| $e$                       | Elevator.                                                                             |
| $r$                       | Руль                                                                                                  |
| $Aero$                    | Аеродинаміка                                                                                          |
| $T$                       | Сила тяги                                                                                             |
| $w$                       | Відносна швидкість повітря                                                                            |
| $x,y,z$                   | Компонент вектора вздовж осі x, y та z.                                               |
| $N,E,D$                   | Компонент вектора вздовж глобального північного, східного та вертикального напрямків. |

<a id="superscripts"></a>

### Верхні індекси / Показники

| Верхні індекси / Показники | Опис                                                                              |
| -------------------------- | --------------------------------------------------------------------------------- |
| $\ell$                     | Місцева система координат Стандарт для змінних, пов'язаних з PX4. |
| $b$                        | Тілесна система координат                                                         |
| $w$                        | Вітрова система координат                                                         |

## Декоратори:

| Декоратор                       | Опис                                   |
| ------------------------------- | -------------------------------------- |
| $()^\*$      | Комплексне відмінювання                |
| $\dot{()}$   | Часова похідна                         |
| $\hat{()}$   | Оцінки                                 |
| $\bar{()}$   | Середнє                                |
| $()^{-1}$    | Обернена матриця                       |
| $()^T$       | Матрична транспозиція. |
| $\tilde{()}$ | Quaternion.            |
