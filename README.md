## Datasets

Para este codigo se requiere bajar los siguientes datasets disponibles públicamente.

Dataset PyShoe: https://github.com/utiasSTARS/pyshoe

Dataset HelmetPoser: https://lqiutong.github.io/HelmetPoser.github.io/

## Estructura de Carpetas

Clonar el repositorio dentro de la carpeta `Codigo` siguiendo la siguiente estructura de carpetas. La ubicación de la carpeta raíz `root` es arbitraria.
En el fichero `Principal.py` se requiere modificar la ruta de la carpeta raíz al equipo correspondiente para acceder a los datasets y ejecutar el código.

<!-- TREEVIEW START -->
```bash
├── root/
│   ├── Codigo/
│   ├── Datasets/
│   │   ├── DatasetPyShoe/
│   │   │   ├── data/
│   │   │   ├── results/
│   │   └── DatasetHelmetPoser/
```
<!-- TREEVIEW END -->

## Scripts

Los scripts de funciones que se manejan son los siguientes.

* `Optimizacion.py`

Fichero que contiene las funciones correspondientes a la optimización de las varianzas de los ruidos del EKF

* `Orientacion.py`

Fichero que contiene las funciones correspondientes a la estimación de la orientación en base a las mediciones de la IMU y cálculo de error en la estimación con respecto a las secuencias de ground-truth.

* `SimuladorIMU.py`

Fichero que contiene un simulador de mediciones de una IMU