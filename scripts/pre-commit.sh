#!/bin/sh

#!/bin/sh
# Part 1
stagedFiles=$(git diff --staged --name-only)
# Part 2
echo "Running spotlessApply. Formatting code..."
./gradlew spotlessApply
# Part 3
for file in $stagedFiles; do
  if test -f "$file"; then
    git add $file
  fi
done
