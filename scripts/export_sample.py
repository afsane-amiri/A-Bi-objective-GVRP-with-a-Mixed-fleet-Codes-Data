from pathlib import Path

from openpyxl import load_workbook


# Paths are relative to the repository root.
workbook_path = Path("Samples.xlsx")
output_dir = Path("data")

sheet_name = "6-10-1-1"
output_path = output_dir / f"Sample{sheet_name}.txt"

# data_only=True reads the stored values rather than Excel formulas.
workbook = load_workbook(workbook_path, data_only=True)
worksheet = workbook[sheet_name]

output_dir.mkdir(exist_ok=True)

with output_path.open("w", encoding="utf-8") as output_file:
    for row in worksheet.iter_rows(values_only=True):
        # Ignore empty cells at the end of each Excel row.
        values = [value for value in row if value is not None]

        if values:
            output_file.write("\t".join(str(value) for value in values))
            output_file.write("\n")

print(f"Exported '{sheet_name}' to '{output_path}'")