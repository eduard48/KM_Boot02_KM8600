# Загрузчк для контроллеров серии КМ тип КМ8600 и КМ2442

>Программа для AtmelStudio v6.2

>>Тип модулей: КM01-8600.M и КМ01-2442.M

Прошить контроллер ATMEGA324p, скомпилированной программой через SPI.

Выключить и включить контроллер. После включения загрузчик ждет начала загрузки по интерфейсу uart

в течении 5 секунд. Если загрузка началась, ждем окончания и перезагружаемся. 

Если загрузка не началась, загрузчик передает управление основной программе контроллера.