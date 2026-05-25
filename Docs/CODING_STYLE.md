# Coding Style Guide

## Objetivo

Definir un conjunto de reglas de estilo para asegurar consistencia y legibilidad en el código del proyecto.

## Idioma

- Todos los identificadores (variables, funciones, structs, macros) deben estar en inglés.
- Los comentarios deben escribirse en español y ser claros y técnicos.

## Nombres y convenciones

- Variables en `snake_case` y descriptivas.
- Estructuras y tipos en `PascalCase` con sufijo `_t`.
- Campos internos de estructuras en `snake_case`.
- Funciones con formato `Modulo_Accion()` que describen claramente su propósito.
- Macros y constantes en mayúsculas.

## Formato de llaves

- La llave de apertura `{` se coloca en la misma línea de la declaración.
- Se utilizan llaves siempre, incluso en bloques simples.

## Comentarios

- Se pueden usar comentarios de línea (`//`) o de bloque (`/* ... */`).
- Priorizar explicar el propósito o la decisión tomada.
- Los comentarios deben ser técnicos y ayudar a entender la intención del código.

## Organización del código

- Separar archivos de interfaz (`.h`) y de implementación (`.c`).
- Los archivos `.h` deben contener declaraciones y definiciones de interfaz sin lógica de implementación.
- Los archivos `.c` deben contener la implementación de funciones y la lógica del módulo.

## Modularidad y calidad

- Priorizar la modularidad y evitar funciones largas.
- Evitar código duplicado.
- Mantener indentación uniforme.
- Mantener una buena estructura general del código.
