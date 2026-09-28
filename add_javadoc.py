from pathlib import Path
import re

# ============================================================
# Configuration
# ============================================================

JAVA_ROOT = Path("TeamCode/src/main/java")

# Add Javadoc to:
#   - classes
#   - interfaces
#   - enums
#   - constructors
#   - public/protected methods
#
# Existing Javadoc is left unchanged.

TYPE_PATTERN = re.compile(
    r'^(\s*)'
    r'(public\s+|protected\s+|private\s+)?'
    r'((?:static\s+|abstract\s+|final\s+)*)'
    r'(class|interface|enum)\s+'
    r'([A-Za-z_]\w*)'
)

METHOD_PATTERN = re.compile(
    r'^(\s*)'
    r'(public|protected)\s+'
    r'((?:static\s+|final\s+|synchronized\s+|abstract\s+)*)'
    r'([A-Za-z_<>\[\], ?.@]+?)\s+'
    r'([A-Za-z_]\w*)\s*'
    r'\((.*)\)'
)

CONSTRUCTOR_PATTERN = re.compile(
    r'^(\s*)'
    r'(public|protected)\s+'
    r'([A-Za-z_]\w*)\s*'
    r'\((.*)\)'
)


def has_javadoc(lines, index):
    """Check whether declaration already has Javadoc above it."""

    i = index - 1

    # Skip annotations and blank lines
    while i >= 0:
        stripped = lines[i].strip()

        if stripped == "":
            i -= 1
            continue

        if stripped.startswith("@"):
            i -= 1
            continue

        break

    if i < 0:
        return False

    # Declaration immediately follows */
    if lines[i].strip().endswith("*/"):
        j = i

        while j >= 0:
            if "/**" in lines[j]:
                return True

            if "/*" in lines[j]:
                return False

            j -= 1

    return False


def get_parameter_names(parameter_text):
    """Extract parameter names from a Java parameter list."""

    parameter_text = parameter_text.strip()

    if not parameter_text:
        return []

    parameters = []
    current = ""
    depth = 0

    # Split on commas, but not commas inside generics
    for char in parameter_text:

        if char == "<":
            depth += 1

        elif char == ">":
            depth -= 1

        if char == "," and depth == 0:
            parameters.append(current.strip())
            current = ""
        else:
            current += char

    if current.strip():
        parameters.append(current.strip())

    names = []

    for parameter in parameters:

        # Remove annotations
        parameter = re.sub(
            r'@\w+(?:\([^)]*\))?\s*',
            '',
            parameter
        )

        # Remove final
        parameter = re.sub(
            r'\bfinal\s+',
            '',
            parameter
        )

        tokens = parameter.split()

        if tokens:
            name = tokens[-1]

            # Handle varargs
            name = name.replace("...", "")

            # Handle array syntax attached to variable
            name = name.replace("[]", "")

            names.append(name)

    return names


def create_javadoc(indent, params=None, has_return=False):
    """Create an empty Javadoc skeleton."""

    result = [
        indent + "/**\n",
        indent + " * \n"
    ]

    if params:
        for param in params:
            result.append(
                indent + f" * @param {param} \n"
            )

    if has_return:
        result.append(
            indent + " * @return \n"
        )

    result.append(
        indent + " */\n"
    )

    return result


def process_java_file(path):

    text = path.read_text(
        encoding="utf-8"
    )

    lines = text.splitlines(
        keepends=True
    )

    output = []

    changed = False

    class_names = []

    i = 0

    while i < len(lines):

        line = lines[i]

        # ----------------------------------------------------
        # Class / interface / enum
        # ----------------------------------------------------

        type_match = TYPE_PATTERN.match(line)

        if type_match:

            indent = type_match.group(1)
            class_name = type_match.group(5)

            class_names.append(class_name)

            if not has_javadoc(lines, i):

                output.extend(
                    create_javadoc(indent)
                )

                changed = True

        # ----------------------------------------------------
        # Constructor
        # ----------------------------------------------------

        constructor_match = CONSTRUCTOR_PATTERN.match(line)

        if constructor_match:

            indent = constructor_match.group(1)
            name = constructor_match.group(3)
            params_text = constructor_match.group(4)

            if name in class_names:

                if not has_javadoc(lines, i):

                    params = get_parameter_names(
                        params_text
                    )

                    output.extend(
                        create_javadoc(
                            indent,
                            params=params
                        )
                    )

                    changed = True

        # ----------------------------------------------------
        # Public / protected method
        # ----------------------------------------------------

        method_match = METHOD_PATTERN.match(line)

        if method_match:

            indent = method_match.group(1)
            return_type = method_match.group(4).strip()
            params_text = method_match.group(6)

            if not has_javadoc(lines, i):

                params = get_parameter_names(
                    params_text
                )

                has_return = (
                    return_type != "void"
                )

                output.extend(
                    create_javadoc(
                        indent,
                        params=params,
                        has_return=has_return
                    )
                )

                changed = True

        output.append(line)

        i += 1

    if changed:

        path.write_text(
            "".join(output),
            encoding="utf-8"
        )

        print(f"UPDATED: {path}")

    else:

        print(f"SKIPPED: {path}")


def main():

    if not JAVA_ROOT.exists():

        print(
            f"ERROR: Cannot find {JAVA_ROOT}"
        )

        print(
            "Run this script from the root "
            "of your FTC project."
        )

        return

    java_files = list(
        JAVA_ROOT.rglob("*.java")
    )

    print(
        f"Found {len(java_files)} Java files."
    )

    for java_file in java_files:

        process_java_file(java_file)

    print()
    print("Finished.")
    print(
        "Review the changes with Git before committing."
    )


if __name__ == "__main__":
    main()