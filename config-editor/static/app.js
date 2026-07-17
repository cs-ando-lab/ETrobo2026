const GATES = {
  red: {
    prefix: "ETRALLY_RED_GATE", css: "red", short: "R",
    label: "赤ゲート", orientation: "horizontal",
  },
  blue: {
    prefix: "ETRALLY_BLUE_GATE", css: "blue", short: "B",
    label: "青ゲート", orientation: "vertical",
  },
  yellow: {
    prefix: "ETRALLY_YELLOW_GATE", css: "yellow", short: "Y",
    label: "黄ゲート", orientation: "horizontal",
  },
};

const form = document.querySelector("#config-form");
const settingsRoot = document.querySelector("#settings");
const categoryNav = document.querySelector("#category-nav");
const grid = document.querySelector("#field-grid");
const search = document.querySelector("#search");
const status = document.querySelector("#status");
const errorBox = document.querySelector("#error");
const saveButton = document.querySelector("#save-button");
const resetButton = document.querySelector("#reset-button");
let config;
let savedValues;
let selectedGate = "red";

function valuesFromConfig() {
  return Object.fromEntries(config.settings.map(setting => [setting.name, setting.value]));
}

function readValues() {
  return Object.fromEntries(
    [...settingsRoot.querySelectorAll("input[data-name]")].map(input => [
      input.dataset.name,
      Number(input.value),
    ]),
  );
}

function groupByCategory(settings) {
  return settings.reduce((groups, setting) => {
    (groups[setting.category] ||= []).push(setting);
    return groups;
  }, {});
}

function renderSettings() {
  const groups = groupByCategory(config.settings);
  categoryNav.innerHTML = Object.keys(groups)
    .map((category, index) => `<a href="#category-${index}">${category}</a>`)
    .join("");
  settingsRoot.innerHTML = Object.entries(groups).map(([category, settings], index) => `
    <section class="settings-section" id="category-${index}" data-category="${category.toLowerCase()}">
      <div class="section-heading">
        <h2>${category}</h2>
        <span>${settings.length}項目</span>
      </div>
      <div class="setting-list">
        ${settings.map(setting => `
          <label class="setting-row"
            data-search="${`${setting.name} ${setting.description} ${category}`.toLowerCase()}">
            <span class="setting-copy">
              <strong>${setting.name}</strong>
              <small>${setting.description || "説明なし"}</small>
            </span>
            <span class="value-field">
              <input
                type="number"
                data-name="${setting.name}"
                data-type="${setting.type}"
                value="${setting.value}"
                step="${setting.type === "float" ? "any" : "1"}"
                required>
              <code>${setting.type}</code>
            </span>
          </label>`).join("")}
      </div>
    </section>`).join("");
}

function gateCoordinates(values) {
  return Object.fromEntries(Object.entries(GATES).map(([color, gate]) => [
    color,
    Object.fromEntries(["left", "right"].map(side => [
      side,
      Object.fromEntries(["row", "col"].map(axis => [
        axis,
        values[`${gate.prefix}_${side.toUpperCase()}_${axis.toUpperCase()}`],
      ])),
    ])),
  ]));
}

function renderGrid(values) {
  const occupied = new Map();
  Object.entries(gateCoordinates(values)).forEach(([color, gate]) => {
    ["left", "right"].forEach(side => {
      const point = gate[side];
      const key = `${point.row}-${point.col}`;
      const entries = occupied.get(key) || [];
      entries.push({ color, side });
      occupied.set(key, entries);
    });
  });
  grid.innerHTML = "";
  for (let row = 1; row <= 5; row += 1) {
    for (let col = 1; col <= 5; col += 1) {
      const cell = document.createElement("button");
      cell.type = "button";
      cell.className = "field-cell";
      cell.dataset.coordinate = `${row},${col}`;
      cell.dataset.row = row;
      cell.dataset.col = col;
      cell.style.setProperty("--point-x", `${((40 + (col - 1) * 118) / 552) * 100}%`);
      cell.style.setProperty("--point-y", `${((40 + (row - 1) * 118) / 552) * 100}%`);
      cell.setAttribute("aria-label", `row ${row}, col ${col}の灰色点に配置`);
      (occupied.get(`${row}-${col}`) || []).forEach(({ color, side }) => {
        const marker = document.createElement("span");
        marker.className = `marker ${GATES[color].css}`;
        marker.textContent = `${GATES[color].short}${side === "left" ? "L" : "R"}`;
        marker.title = `${GATES[color].label} ${side === "left" ? "左脚" : "右脚"}`;
        cell.append(marker);
      });
      grid.append(cell);
    }
  }
}

function settingInput(name) {
  return settingsRoot.querySelector(`input[data-name="${name}"]`);
}

function placementAt(row, col) {
  const horizontal = GATES[selectedGate].orientation === "horizontal";
  const left = {
    row: horizontal ? row : Math.min(row, 4),
    col: horizontal ? Math.min(col, 4) : col,
  };
  const right = {
    row: horizontal ? row : left.row + 1,
    col: horizontal ? left.col + 1 : col,
  };
  return { left, right };
}

function placeGate(row, col) {
  const gate = GATES[selectedGate];
  const coordinates = placementAt(row, col);
  ["left", "right"].forEach(side => {
    ["row", "col"].forEach(axis => {
      settingInput(`${gate.prefix}_${side.toUpperCase()}_${axis.toUpperCase()}`).value =
        coordinates[side][axis];
    });
  });
  updateState();
}

function clearPlacementPreview() {
  grid.querySelectorAll(".placement-preview").forEach(cell => {
    cell.classList.remove("placement-preview", "preview-red", "preview-blue", "preview-yellow");
  });
}

function showPlacementPreview(row, col) {
  clearPlacementPreview();
  Object.values(placementAt(row, col)).forEach(point => {
    const cell = grid.querySelector(`[data-row="${point.row}"][data-col="${point.col}"]`);
    cell?.classList.add("placement-preview", `preview-${selectedGate}`);
  });
}

function selectGate(gate) {
  selectedGate = gate;
  document.querySelectorAll(".gate-choice").forEach(button => {
    button.classList.toggle("active", button.dataset.gate === selectedGate);
  });
  const direction = GATES[selectedGate].orientation === "horizontal" ? "横向き" : "縦向き";
  document.querySelector("#placement-hint").textContent =
    `${GATES[selectedGate].label}を${direction}に配置します`;
}

function validate(values) {
  const occupied = new Map();
  for (const [color, gate] of Object.entries(gateCoordinates(values))) {
    for (const point of Object.values(gate)) {
      if (point.row < 1 || point.row > 5 || point.col < 1 || point.col > 5) {
        return `${GATES[color].label}のrowとcolは1〜5で指定してください。`;
      }
      const key = `${point.row}-${point.col}`;
      if (occupied.has(key)) {
        return `${key.replace("-", "行・")}列でゲート同士が重なっています。`;
      }
      occupied.set(key, color);
    }
    const distance =
      Math.abs(gate.left.row - gate.right.row) +
      Math.abs(gate.left.col - gate.right.col);
    if (distance !== 1) {
      return `${GATES[color].label}の左右の脚は隣り合うマスに配置してください。`;
    }
    const shouldBeHorizontal = GATES[color].orientation === "horizontal";
    if (shouldBeHorizontal && gate.left.row !== gate.right.row) {
      return `${GATES[color].label}は横向きに配置してください。`;
    }
    if (!shouldBeHorizontal && gate.left.col !== gate.right.col) {
      return `${GATES[color].label}は縦向きに配置してください。`;
    }
  }
  const invalidInput = settingsRoot.querySelector("input:invalid");
  return invalidInput ? `${invalidInput.dataset.name}の値を確認してください。` : "";
}

function updateState() {
  const values = readValues();
  renderGrid(values);
  const validationError = validate(values);
  const changed = JSON.stringify(values) !== JSON.stringify(savedValues);
  errorBox.hidden = !validationError;
  errorBox.textContent = validationError;
  saveButton.disabled = Boolean(validationError) || !changed;
  resetButton.disabled = !changed;
  status.textContent = validationError
    ? "入力内容を確認してください"
    : changed ? "未保存の変更があります" : "保存済み";
  settingsRoot.querySelectorAll(".setting-row").forEach(row => {
    const input = row.querySelector("input");
    row.classList.toggle("changed", input && Number(input.value) !== savedValues[input.dataset.name]);
  });
}

function filterSettings() {
  const query = search.value.trim().toLowerCase();
  settingsRoot.querySelectorAll(".setting-row").forEach(row => {
    row.hidden = Boolean(query) && !row.dataset.search.includes(query);
  });
  settingsRoot.querySelectorAll(".settings-section").forEach(section => {
    section.hidden = ![...section.querySelectorAll(".setting-row")].some(row => !row.hidden);
  });
}

async function loadConfig() {
  try {
    const response = await fetch("/api/config");
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "設定を読み込めませんでした");
    config = data;
    savedValues = valuesFromConfig();
    renderSettings();
    renderGrid(savedValues);
    status.textContent = `${config.settings.length}件を読み込みました`;
    settingsRoot.addEventListener("input", updateState);
  } catch (error) {
    errorBox.hidden = false;
    errorBox.textContent = error.message;
    status.textContent = "読み込みエラー";
  }
}

resetButton.addEventListener("click", () => {
  settingsRoot.querySelectorAll("input[data-name]").forEach(input => {
    input.value = savedValues[input.dataset.name];
  });
  updateState();
});

search.addEventListener("input", filterSettings);
grid.addEventListener("click", event => {
  const cell = event.target.closest(".field-cell");
  if (cell) placeGate(Number(cell.dataset.row), Number(cell.dataset.col));
});
grid.addEventListener("mouseover", event => {
  const cell = event.target.closest(".field-cell");
  if (cell) showPlacementPreview(Number(cell.dataset.row), Number(cell.dataset.col));
});
grid.addEventListener("focusin", event => {
  const cell = event.target.closest(".field-cell");
  if (cell) showPlacementPreview(Number(cell.dataset.row), Number(cell.dataset.col));
});
grid.addEventListener("mouseleave", clearPlacementPreview);
grid.addEventListener("focusout", event => {
  if (!grid.contains(event.relatedTarget)) clearPlacementPreview();
});
document.querySelectorAll(".gate-choice").forEach(button => {
  button.addEventListener("click", () => selectGate(button.dataset.gate));
});

form.addEventListener("submit", async event => {
  event.preventDefault();
  updateState();
  if (saveButton.disabled) return;
  saveButton.disabled = true;
  status.textContent = "保存しています…";
  try {
    const response = await fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ values: readValues() }),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "保存できませんでした");
    config = { version: data.version, settings: data.settings };
    savedValues = valuesFromConfig();
    updateState();
    status.textContent = data.message;
    errorBox.hidden = true;
  } catch (error) {
    errorBox.hidden = false;
    errorBox.textContent = error.message;
    status.textContent = "保存エラー";
    saveButton.disabled = false;
  }
});

loadConfig();
