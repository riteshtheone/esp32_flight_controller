#ifndef HTMLPAGE_H
#define HTMLPAGE_H

#include "Arduino.h"

const char page[] PROGMEM = R"rawLiteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, user-scalable=no, initial-scale=1.0, maximum-scale=1.0, minimum-scale=1.0">
    <meta http-equiv="X-UA-Compatible" content="ie=edge">
    <title>pid setup</title>
    <style>
        body { font-family: Arial, sans-serif; }
        .container { width: 95%; margin: auto; }
        .container h2 { margin-bottom: 20px; text-align: center; }
        .form-group { margin-bottom: 15px; display: flex; align-items: center; }
        .form-group label { flex: 2; margin-right: 5px; font-weight: bold; }
        .form-group input[type="number"] { flex: 1; padding: 8px; font-size: 16px; border: 1px solid #ccc; border-radius: 4px; box-sizing: border-box; }
        button { flex: 1; background-color: #4CAF50; color: white; padding: 8px 15px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; }
        button:hover { background-color: #45a049; }
    </style>
</head>
<body>
    <div class="container">
        <h2>PID Controller Settings</h2>
        <form id="myForm" method="post" novalidate>
            <div class="form-group">
                <label for="rollPitchP">Roll/Pitch(1.3) P:</label>
                <input type="number" id="rollPitchP" name="rpP" value="0.0">
            </div>
            <div class="form-group">
                <label for="rollPitchI">Roll/Pitch(0.04) I:</label>
                <input type="number" id="rollPitchI" name="rpI" value="0.0">
            </div>
            <div class="form-group">
                <label for="rollPitchD">Roll/Pitch(18.0) D:</label>
                <input type="number" id="rollPitchD" name="rpD" value="0.0">
            </div>
            <div class="form-group">
                <label for="yawP">Yaw(4.0) P:</label>
                <input type="number" id="yawP" name="yP" value="0.0">
            </div>
            <div class="form-group">
                <label for="yawI">Yaw(0.02) I:</label>
                <input type="number" id="yawI" name="yI" value="0.0">
            </div>
            <div class="form-group">
                <label for="yawD">Yaw(0) D:</label>
                <input type="number" id="yawD" name="yD" value="0.0">
            </div>
            <button type="submit">Submit</button>
        </form>
    </div>
    <script>
        const removeTrailingZeros = (str) => str.replace(/(\.\d*?[1-9])0+$/g, '$1').replace(/\.$/,'')
        document.addEventListener('DOMContentLoaded', () => {
            fetch(`${window.location.origin}/pid`)
                .then(response => response.json())
                .then(data => {
                    document.getElementById('rollPitchP').value = removeTrailingZeros(data.rpP)
                    document.getElementById('rollPitchI').value = removeTrailingZeros(data.rpI)
                    document.getElementById('rollPitchD').value = removeTrailingZeros(data.rpD)
                    document.getElementById('yawP').value = removeTrailingZeros(data.yP)
                    document.getElementById('yawI').value = removeTrailingZeros(data.yI)
                    document.getElementById('yawD').value = removeTrailingZeros(data.yD)
                }).catch(error => console.error('Error fetching data:', error))
        })
    </script>
</body>
</html>
)rawLiteral";
#endif //HTMLPAGE_H
