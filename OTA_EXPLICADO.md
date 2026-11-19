# Cómo funciona el sistema OTA con GitHub

Este documento explica el **por qué** de cada pieza del sistema, no solo el cómo.

---

## El problema que resuelve

Sin OTA, para actualizar el firmware del ESP32 tienes que:
1. Conectar el cable USB
2. Abrir la terminal
3. Correr `idf.py flash`

Con OTA, el ESP32 se actualiza **solo por WiFi** mientras está montado en el equipo. Solo necesitas subir el nuevo firmware a GitHub.

---

## Por qué `.\publish.ps1` y no hacerlo a mano

Actualizar implica varios pasos que deben hacerse **en orden y sin errores**:

1. Compilar el firmware
2. Subir el `.bin` a GitHub Releases
3. Actualizar el `version.json` en el repositorio
4. Hacer commit y push

Si lo haces a mano puedes olvidar algún paso. Por ejemplo, si subes el `.bin` pero no actualizas el `version.json`, el ESP32 nunca sabrá que hay una versión nueva. El script `publish.ps1` hace todo esto de un solo comando:

```powershell
.\publish.ps1
```

---

## Cómo sabe el publish.ps1 qué versión poner

El script lee la versión directamente del `CMakeLists.txt`:

```powershell
$match = Select-String -Path "CMakeLists.txt" -Pattern 'project\(\w+ VERSION ([\d.]+)\)'
$version = $match.Matches[0].Groups[1].Value
```

Esto busca la línea:
```cmake
project(WP_R_V9_UP VERSION 1.4.0)
                           ↑ extrae esto
```

Así no tienes que escribir la versión en dos lugares. La cambias una sola vez en `CMakeLists.txt` y el script la toma automáticamente.

---

## Por qué el archivo se llama `WP_R_V9_UP.bin` tanto en GitHub como en tu PC

El ESP32 tiene grabada en el código una URL fija que **nunca cambia**:

```c
#define OTA_FIRMWARE_URL \
  "https://github.com/SrMuchachita/Muchachota_R/releases/latest/download/WP_R_V9_UP.bin"
```

Cuando el ESP32 quiere actualizarse, va a esa URL y descarga lo que encuentre ahí. Si el archivo en GitHub tuviera un nombre diferente cada versión (`WP_R_V9_UP_v1.3.bin`, `WP_R_V9_UP_v1.4.bin`, etc.), tendrías que cambiar la URL en el código con cada versión, lo que arruinaría la gracia del OTA.

La solución es que el nombre del archivo **se mantenga fijo en GitHub** (`WP_R_V9_UP.bin`), igual que en tu PC. `publish.ps1` lo sube tal cual, sin renombrarlo:

```powershell
gh release create "v$version" "build/WP_R_V9_UP.bin" `
    --title "v$version" `
    --notes "$Notes"
```

Como el nombre nunca cambia entre versiones, el ESP32 siempre descarga `WP_R_V9_UP.bin` de `/releases/latest/download/` y siempre recibe el firmware correcto.

---

## Por qué hay dos URLs en el código del ESP32

```c
#define OTA_VERSION_URL  "https://raw.githubusercontent.com/.../version.json"
#define OTA_FIRMWARE_URL "https://github.com/.../releases/latest/download/WP_R_V9_UP.bin"
```

Son dos pasos distintos con propósitos distintos:

### `version.json` — para decidir si actualizar

Antes de descargar el firmware (que pesa ~1MB), el ESP32 consulta el `version.json`:

```json
{"version":"1.4.0"}
```

Compara ese número con el que tiene grabado el firmware que está corriendo. Si son iguales, no hace nada. Si son distintos, recién ahí descarga el `.bin`.

Esto ahorra ancho de banda y tiempo. En lugar de bajar 1MB cada 20 segundos para comparar, solo baja unos pocos bytes del JSON.

### `WP_R_V9_UP.bin` — para actualizar

Solo se descarga cuando se confirmó que hay versión nueva. Puede pesar más de 1MB, así que este paso tarda más.

---

## Por qué se usa `/releases/latest/download/` y no otra URL

GitHub tiene varios tipos de URLs para archivos:

| URL | Qué hace |
|-----|----------|
| `.../releases/download/v1.4.0/firmware.bin` | Descarga versión específica |
| `.../releases/latest/download/firmware.bin` | Descarga **siempre la última versión** |

Con `/latest/` no necesitas actualizar la URL en el firmware cada vez que subes una versión nueva. GitHub automáticamente redirige al release más reciente que tenga el archivo `firmware.bin`.

---

## Por qué se necesita `esp_crt_bundle_attach`

GitHub usa HTTPS (conexión cifrada). Para que el ESP32 confíe en el certificado de GitHub, necesita conocer las autoridades certificadoras (CAs) que emitieron ese certificado.

```c
esp_http_client_config_t cfg = {
    .crt_bundle_attach = esp_crt_bundle_attach,  // adjunta el paquete de CAs
};
```

`esp_crt_bundle_attach` adjunta un paquete con más de 100 CAs reconocidas (similar al que tiene tu navegador). Sin esto, el ESP32 rechaza la conexión con un error SSL porque no puede verificar que realmente está hablando con GitHub.

Esto requiere activar en `sdkconfig.defaults`:
```
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEFAULT_FULL=y
```

---

## Por qué `buffer_size = 8192` en la descarga del firmware

GitHub no entrega el archivo directamente. Cuando el ESP32 pide `firmware.bin`, GitHub responde con una **redirección** (HTTP 302) a otro servidor (el CDN de GitHub). Esa respuesta de redirección tiene cabeceras HTTP muy largas.

Si el buffer es muy pequeño, el ESP32 no puede leer las cabeceras completas y falla con error "Out of buffer". Con 8192 bytes hay espacio suficiente para las cabeceras de GitHub.

```c
esp_http_client_config_t http_cfg = {
    .buffer_size    = 8192,   // para las cabeceras de redirección de GitHub
    .buffer_size_tx = 4096,
};
```

---

## Por qué `CONFIG_MBEDTLS_DYNAMIC_BUFFER=y`

Por defecto, mbedTLS reserva los buffers TLS en el momento de inicialización, aunque no los esté usando. En un proyecto con muchos componentes (Modbus, MCPWM, I2C, etc.) esto puede dejar sin heap al ESP32.

Con `DYNAMIC_BUFFER`, mbedTLS solo reserva memoria cuando realmente la necesita (durante la conexión HTTPS) y la libera al terminar. Esto permite que el resto del firmware siga funcionando normalmente.

---

## Por qué el OTA task tiene stack de 16384 bytes

Las operaciones TLS (handshake, verificación de certificados, descifrado) requieren mucha memoria de **stack** además de heap. Con 8192 bytes el stack puede desbordarse durante el handshake SSL, causando el error `mbedtls_ssl_setup returned -0x008D`.

Con 16384 bytes (16KB) hay margen suficiente para todas las operaciones criptográficas.

```c
xTaskCreate(ota_task, "ota_task", 16384, NULL, 5, NULL);
```

---

## Flujo completo resumido

```
Tú cambias VERSION en CMakeLists.txt
        ↓
.\publish.ps1
        ↓
idf.py build → genera WP_R_V9_UP.bin
        ↓
gh release create → sube como firmware.bin en GitHub
        ↓
publish.ps1 actualiza version.json → git push
        ↓
ESP32 (cada 20s) consulta version.json
        ↓
¿Versión diferente? → descarga firmware.bin → graba en flash → reinicia
        ↓
ESP32 corre el nuevo firmware
```
