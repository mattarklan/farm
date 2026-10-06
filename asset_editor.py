import sys
import os
import glob
import yaml

from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QMainWindow,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QComboBox,
    QPushButton,
    QLineEdit,
    QSpinBox,
    QDoubleSpinBox,
    QListWidget,
    QGroupBox,
    QFormLayout,
    QMessageBox,
    QTextEdit,
    QListWidgetItem,
)

from PySide6.QtCore import Qt


REGISTRY_PATH = "config/registry"


# ------------------------------------------------------------
# YAML handling
# ------------------------------------------------------------

class RegistryManager:

    def __init__(self, folder):
        self.folder = folder
        self.current_file = None
        self.data = None

    def available_registries(self):
        files = glob.glob(
            os.path.join(self.folder, "*.yaml")
        )
        return sorted([
            os.path.splitext(os.path.basename(f))[0]
            for f in files
        ])

    def load(self, name):
        path = os.path.join(
            self.folder,
            name + ".yaml"
        )

        self.current_file = path

        with open(path, "r") as f:
            self.data = yaml.safe_load(f)

        return self.data


    def save(self):
        with open(self.current_file, "w") as f:
            yaml.safe_dump(
                self.data,
                f,
                sort_keys=False,
                default_flow_style=False
            )


# ------------------------------------------------------------
# Schema inference
# ------------------------------------------------------------

class SchemaInferer:

    @staticmethod
    def get_first_asset(registry):

        # water:
        #   tank_1:
        #      fields

        registry_name = list(registry.keys())[0]

        assets = registry[registry_name]

        asset_id = list(assets.keys())[0]

        return assets[asset_id]


    @staticmethod
    def infer(value):

        if isinstance(value, bool):
            return "bool"

        if isinstance(value, int):
            return "int"

        if isinstance(value, float):
            return "float"

        if isinstance(value, str):
            return "str"

        if isinstance(value, list):

            if len(value) == 2:
                if all(
                    isinstance(x, float)
                    for x in value
                ):
                    return "gps"

            return "list"

        if isinstance(value, dict):
            return "dict"

        return "unknown"


# ------------------------------------------------------------
# Dynamic widgets
# ------------------------------------------------------------

class ListEditor(QWidget):

    def __init__(self, values=None):
        super().__init__()

        layout = QVBoxLayout()

        self.listbox = QListWidget()

        for v in values or []:
            self.listbox.addItem(str(v))

        buttons = QHBoxLayout()

        add = QPushButton("+")
        remove = QPushButton("-")

        add.clicked.connect(self.add_item)
        remove.clicked.connect(self.remove_item)

        buttons.addWidget(add)
        buttons.addWidget(remove)

        layout.addWidget(self.listbox)
        layout.addLayout(buttons)

        self.setLayout(layout)


    def add_item(self):
        self.listbox.addItem("")


    def remove_item(self):
        row = self.listbox.currentRow()

        if row >= 0:
            self.listbox.takeItem(row)


    def value(self):
        return [
            self.listbox.item(i).text()
            for i in range(self.listbox.count())
        ]


class GPSWidget(QWidget):

    def __init__(self, value=None):
        super().__init__()

        layout = QHBoxLayout()

        self.lat = QDoubleSpinBox()
        self.lon = QDoubleSpinBox()

        self.lat.setDecimals(8)
        self.lon.setDecimals(8)

        self.lat.setRange(-90,90)
        self.lon.setRange(-180,180)

        if value:
            self.lat.setValue(value[0])
            self.lon.setValue(value[1])

        layout.addWidget(
            QLabel("Lat")
        )

        layout.addWidget(
            self.lat
        )

        layout.addWidget(
            QLabel("Lon")
        )

        layout.addWidget(
            self.lon
        )

        self.setLayout(layout)


    def value(self):

        return [
            self.lat.value(),
            self.lon.value()
        ]


# ------------------------------------------------------------
# Form generator
# ------------------------------------------------------------

class DynamicForm(QWidget):

    def __init__(self, asset):
        super().__init__()

        self.widgets = {}

        self.layout = QFormLayout()

        self.build(
            asset,
            self.layout
        )

        self.setLayout(
            self.layout
        )


    def build(self, data, layout, prefix=""):

        for key, value in data.items():

            path = (
                f"{prefix}.{key}"
                if prefix
                else key
            )

            kind = SchemaInferer.infer(value)


            if kind == "str":

                w = QLineEdit()
                w.setText(value)

            elif kind == "int":

                w = QSpinBox()
                w.setRange(
                    -999999,
                    999999
                )
                w.setValue(value)

            elif kind == "float":

                w = QDoubleSpinBox()
                w.setDecimals(8)
                w.setValue(value)

            elif kind == "gps":

                w = GPSWidget(value)

            elif kind == "list":

                w = ListEditor(value)

            elif kind == "dict":

                box = QGroupBox(key)

                sub = QFormLayout()

                self.build(
                    value,
                    sub,
                    path
                )

                box.setLayout(sub)

                w = box


            else:
                continue


            self.widgets[path] = w

            layout.addRow(
                QLabel(key),
                w
            )


    def value(self):

        result = {}

        for path, widget in self.widgets.items():

            if isinstance(widget, QLineEdit):
                val = widget.text()

            elif isinstance(widget, QSpinBox):
                val = widget.value()

            elif isinstance(widget, QDoubleSpinBox):
                val = widget.value()

            elif isinstance(widget, GPSWidget):
                val = widget.value()

            elif isinstance(widget, ListEditor):
                val = widget.value()

            else:
                continue


            result[path] = val


        return result

# ------------------------------------------------------------
# Main application window
# ------------------------------------------------------------

class AssetRegistryEditor(QMainWindow):

    def __init__(self):

        super().__init__()

        self.setWindowTitle(
            "Farm Asset Registry Editor"
        )

        self.resize(
            900,
            700
        )

        self.registry = RegistryManager(
            REGISTRY_PATH
        )

        self.current_registry_name = None
        self.form = None
        self.asset_preview = None

        self.root = QWidget()

        self.layout = QVBoxLayout()

        # Registry selector

        top = QHBoxLayout()

        top.addWidget(
            QLabel("Registry:")
        )

        self.registry_box = QComboBox()

        self.registry_box.addItems(
            self.registry.available_registries()
        )

        self.registry_box.currentTextChanged.connect(
            self.load_registry
        )

        top.addWidget(
            self.registry_box
        )

        self.layout.addLayout(
            top
        )


        # Asset ID

        id_row = QHBoxLayout()

        id_row.addWidget(
            QLabel("New Asset ID:")
        )

        self.asset_id = QLineEdit()

        id_row.addWidget(
            self.asset_id
        )

        self.layout.addLayout(
            id_row
        )


        # Dynamic form container

        self.form_container = QWidget()

        self.form_layout = QVBoxLayout()

        self.form_container.setLayout(
            self.form_layout
        )

        self.layout.addWidget(
            self.form_container
        )


        # Preview

        self.layout.addWidget(
            QLabel("YAML Preview")
        )

        self.asset_preview = QTextEdit()

        self.asset_preview.setReadOnly(
            True
        )

        self.layout.addWidget(
            self.asset_preview
        )


        # Buttons

        buttons = QHBoxLayout()

        preview = QPushButton(
            "Preview"
        )

        save = QPushButton(
            "Save Asset"
        )

        preview.clicked.connect(
            self.update_preview
        )

        save.clicked.connect(
            self.save_asset
        )


        buttons.addWidget(
            preview
        )

        buttons.addWidget(
            save
        )

        self.layout.addLayout(
            buttons
        )


        self.root.setLayout(
            self.layout
        )

        self.setCentralWidget(
            self.root
        )


        if self.registry_box.count():

            self.load_registry(
                self.registry_box.currentText()
            )


    # --------------------------------------------------------
    # Load selected YAML
    # --------------------------------------------------------

    def load_registry(self, name):

        if not name:
            return

        self.current_registry_name = name

        data = self.registry.load(
            name
        )

        asset = SchemaInferer.get_first_asset(
            data
        )


        # Clear previous form

        while self.form_layout.count():

            item = self.form_layout.takeAt(0)

            if item.widget():
                item.widget().deleteLater()


        self.form = DynamicForm(
            asset
        )


        self.form_layout.addWidget(
            self.form
        )



    # --------------------------------------------------------
    # Convert flat widget paths into nested dict
    # --------------------------------------------------------

    def build_asset(self):

        flat = self.form.value()

        asset = {}


        for path, value in flat.items():

            parts = path.split(".")

            current = asset

            for part in parts[:-1]:

                if part not in current:
                    current[part] = {}

                current = current[part]


            current[parts[-1]] = value


        asset["id"] = (
            self.asset_id.text()
            .strip()
        )

        return asset



    # --------------------------------------------------------
    # Preview YAML
    # --------------------------------------------------------

    def update_preview(self):

        asset = self.build_asset()

        output = {
            self.asset_id.text():
            asset
        }


        self.asset_preview.setText(
            yaml.safe_dump(
                output,
                sort_keys=False
            )
        )



    # --------------------------------------------------------
    # Save new asset
    # --------------------------------------------------------

    def save_asset(self):

        asset_id = (
            self.asset_id.text()
            .strip()
        )

        if not asset_id:

            QMessageBox.warning(
                self,
                "Missing ID",
                "Asset ID required"
            )

            return


        asset = self.build_asset()


        registry_name = list(
            self.registry.data.keys()
        )[0]


        assets = self.registry.data[
            registry_name
        ]


        if asset_id in assets:

            QMessageBox.warning(
                self,
                "Duplicate",
                "Asset ID already exists"
            )

            return


        assets[asset_id] = asset


        self.registry.save()


        QMessageBox.information(
            self,
            "Saved",
            f"{asset_id} added"
        )



# ------------------------------------------------------------
# Program entry
# ------------------------------------------------------------

def main():

    app = QApplication(sys.argv)

    window = AssetRegistryEditor()

    window.show()

    sys.exit(
        app.exec()
    )


if __name__ == "__main__":
    main()