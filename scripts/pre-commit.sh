#!/bin/sh

echo "*****Running formatter******"

./gradlew spotlessApply

echo "*****Done with formatter******"

exit $status
