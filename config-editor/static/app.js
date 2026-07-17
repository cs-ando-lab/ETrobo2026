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
const diffDialog = document.querySelector("#diff-dialog");
const diffList = document.querySelector("#diff-list");
const diffCount = document.querySelector("#diff-count");
const diffCancelButton = document.querySelector("#diff-cancel");
const diffConfirmButton = document.querySelector("#diff-confirm");
const historyList = document.querySelector("#history-list");
let config;
let savedValues;
let historySnapshots = [];
let selectedGate = "red";
let selectedCategoryIndex = 0;

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

function displayMetadata(description) {
  const unitMatch = description.match(/\[([^\]]+)\]/);
  return {
    unit: unitMatch ? unitMatch[1] : "",
    description: unitMatch ? description.replace(unitMatch[0], "").trim() : description,
  };
}

function renderSettings() {
  const groups = groupByCategory(config.settings);
  categoryNav.innerHTML = Object.keys(groups)
    .map((category, index) => `
      <button
        type="button"
        data-category-index="${index}"
        aria-pressed="${index === selectedCategoryIndex}">
        ${category}<span>${groups[category].length}</span>
      </button>`)
    .join("");
  settingsRoot.innerHTML = Object.entries(groups).map(([category, settings], index) => `
    <section
      class="settings-section"
      id="category-${index}"
      data-category="${category.toLowerCase()}"
      data-category-index="${index}"
      ${index === selectedCategoryIndex ? "" : "hidden"}>
      <div class="section-heading">
        <h2>${category}</h2>
        <span>${settings.length}項目</span>
      </div>
      <div class="setting-list">
        ${settings.map(setting => {
          const metadata = displayMetadata(setting.description);
          return `
          <label class="setting-row"
            data-search="${`${setting.name} ${setting.description} ${category}`.toLowerCase()}">
            <span class="setting-copy">
              <strong>${setting.name}</strong>
              <small>${metadata.description || "説明なし"}</small>
            </span>
            <span class="value-field">
              <code class="type-label">${setting.type}</code>
              <input
                type="number"
                data-name="${setting.name}"
                data-type="${setting.type}"
                value="${setting.value}"
                step="${setting.type === "float" ? "any" : "1"}"
                required>
              <span class="unit-label">${metadata.unit}</span>
            </span>
          </label>`;
        }).join("")}
      </div>
      <p class="empty-results" hidden>このカテゴリに一致する設定はありません。</p>
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

function settingByName(name) {
  return config.settings.find(setting => setting.name === name);
}

function formatDiffValue(setting, value) {
  const metadata = displayMetadata(setting.description);
  return metadata.unit ? `${value} ${metadata.unit}` : `${value}`;
}

function changedNamesAgainstSaved(values) {
  return Object.keys(values).filter(name => values[name] !== savedValues[name]);
}

// 変更された項目だけを一覧化し、ダイアログに描画する。戻り値は変更件数。
function renderDiff(values) {
  const changedNames = changedNamesAgainstSaved(values);
  diffCount.textContent = `${changedNames.length}件の変更`;
  diffList.innerHTML = changedNames.map(name => {
    const setting = settingByName(name);
    return `
      <div class="diff-row">
        <span class="diff-copy">
          <strong>${setting.name}</strong>
          <small>${setting.category}</small>
        </span>
        <span class="diff-values">
          <span class="diff-old">${formatDiffValue(setting, savedValues[name])}</span>
          <span class="diff-arrow" aria-hidden="true">→</span>
          <span class="diff-new">${formatDiffValue(setting, values[name])}</span>
        </span>
      </div>`;
  }).join("");
  return changedNames.length;
}

// 差分プレビューを表示し、確定されたらactionにvaluesを渡して実行する。変更がなければ何もしない。
function requestConfirmation(values, action) {
  if (renderDiff(values) === 0) return;
  diffDialog.showModal();
  diffConfirmButton.onclick = () => {
    diffDialog.close();
    action(values);
  };
}

async function commitSave(values) {
  saveButton.disabled = true;
  status.textContent = "保存しています…";
  try {
    const response = await fetch("/api/config", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ values }),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "保存できませんでした");
    config = { version: data.version, settings: data.settings };
    savedValues = valuesFromConfig();
    updateState();
    status.textContent = data.message;
    errorBox.hidden = true;
    loadHistory();
  } catch (error) {
    errorBox.hidden = false;
    errorBox.textContent = error.message;
    status.textContent = "保存エラー";
    saveButton.disabled = false;
  }
}

function formatTimestamp(iso) {
  const date = new Date(iso);
  if (Number.isNaN(date.getTime())) return iso;
  return date.toLocaleString("ja-JP", {
    year: "numeric", month: "2-digit", day: "2-digit",
    hour: "2-digit", minute: "2-digit", second: "2-digit",
  });
}

function valuesFromSnapshot(snapshot) {
  return Object.fromEntries(snapshot.settings.map(setting => [setting.name, setting.value]));
}

function renderHistory() {
  if (!historySnapshots.length) {
    historyList.innerHTML = `<p class="history-empty">まだ変更履歴がありません。保存すると自動的に記録されます。</p>`;
    return;
  }
  historyList.innerHTML = historySnapshots.map(snapshot => {
    const values = valuesFromSnapshot(snapshot);
    const changeCount = changedNamesAgainstSaved(values).length;
    return `
      <div class="history-row">
        <span class="history-copy">
          <strong>${formatTimestamp(snapshot.timestamp)}</strong>
          <small>${changeCount > 0 ? `現在の内容と${changeCount}件差分` : "現在の内容と同じ"}</small>
        </span>
        <button
          type="button"
          class="secondary history-restore"
          data-id="${snapshot.id}"
          ${changeCount === 0 ? "disabled" : ""}>
          この内容に戻す
        </button>
      </div>`;
  }).join("");
}

async function loadHistory() {
  try {
    const response = await fetch("/api/history");
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "変更履歴を読み込めませんでした");
    historySnapshots = data.snapshots;
    renderHistory();
  } catch (error) {
    historyList.innerHTML = `<p class="history-empty">変更履歴を読み込めませんでした。</p>`;
  }
}

historyList.addEventListener("click", event => {
  const button = event.target.closest(".history-restore");
  if (!button || button.disabled) return;
  const snapshot = historySnapshots.find(item => item.id === button.dataset.id);
  if (!snapshot) return;
  requestConfirmation(valuesFromSnapshot(snapshot), commitSave);
});

function filterSettings() {
  const query = search.value.trim().toLowerCase();
  settingsRoot.querySelectorAll(".setting-row").forEach(row => {
    row.hidden = Boolean(query) && !row.dataset.search.includes(query);
  });
  settingsRoot.querySelectorAll(".settings-section").forEach(section => {
    const hasMatch = [...section.querySelectorAll(".setting-row")].some(row => !row.hidden);
    const index = Number(section.dataset.categoryIndex);
    section.hidden = index !== selectedCategoryIndex;
    section.querySelector(".empty-results").hidden = hasMatch;
    const tab = categoryNav.querySelector(`[data-category-index="${index}"]`);
    tab.classList.toggle("no-match", Boolean(query) && !hasMatch);
  });
}

function selectCategory(index) {
  selectedCategoryIndex = index;
  categoryNav.querySelectorAll("[data-category-index]").forEach(tag => {
    const selected = Number(tag.dataset.categoryIndex) === selectedCategoryIndex;
    tag.setAttribute("aria-pressed", String(selected));
  });
  filterSettings();
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
    loadHistory();
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
categoryNav.addEventListener("click", event => {
  const tab = event.target.closest("[data-category-index]");
  if (tab) selectCategory(Number(tab.dataset.categoryIndex));
});
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

form.addEventListener("submit", event => {
  event.preventDefault();
  updateState();
  if (saveButton.disabled) return;
  requestConfirmation(readValues(), commitSave);
});

diffCancelButton.addEventListener("click", () => diffDialog.close());

loadConfig();
