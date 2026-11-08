// Copyright (c) 2026, ROS Mobile Robots
// SPDX-License-Identifier: BSD-3-Clause
// Validate binary glTF files with the Khronos glTF-Validator.
// Usage: node validate.js FILE.glb...   Exit code 1 if a file has errors or warnings.
'use strict';

const fs = require('fs');
const path = require('path');
const validator = require('gltf-validator');

async function main(files) {
  let failed = 0;
  for (const file of files) {
    const report = await validator.validateBytes(new Uint8Array(fs.readFileSync(file)));
    const { numErrors, numWarnings, numInfos, messages } = report.issues;
    console.log(`${path.basename(file)}: ${numErrors} errors, ${numWarnings} warnings, ` +
                `${numInfos} infos`);
    for (const message of messages.filter((m) => m.severity <= 1)) {
      console.log(`  ${message.code} ${message.pointer || ''}: ${message.message}`);
    }
    if (numErrors + numWarnings > 0) {
      failed += 1;
    }
  }
  console.log(`${files.length} files validated, ${failed} with errors or warnings`);
  return failed ? 1 : 0;
}

main(process.argv.slice(2)).then((code) => process.exit(code), (error) => {
  console.error(error);
  process.exit(2);
});
