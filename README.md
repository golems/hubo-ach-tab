hubo-ach-tab
============

A tab that allows the
[DART](https://github.com/golems/dart)/[GRIP](https://github.com/golems/grip)
simulation environment to emulate a Hubo robot by pretending to be the
hardware daemon from
[hubo-ach](https://github.com/hubo/hubo-ach). When loaded, this tab
attaches to the necessary state and command channels, feeding commands
to the simulated hubo in DART/GRIP and publishing state updates as
appropriate.