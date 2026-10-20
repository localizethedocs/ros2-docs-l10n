"use strict";

const _is_file_uri = (uri) => uri.startsWith("file:/");
const _IS_LOCAL = _is_file_uri(window.location.href);
const _CURRENT_LANGUAGE = CURRENT_OPTIONS.CURRENT_LANGUAGE;   /* Get from current.js */
const _CURRENT_VERSION = CURRENT_OPTIONS.CURRENT_VERSION;     /* Get from current.js */
const _CONFIG_LANGUAGES = CONFIG_OPTIONS.CONFIG_LANGUAGES;    /* Get from config.js */
const _CONFIG_VERSIONS = CONFIG_OPTIONS.CONFIG_VERSIONS;      /* Get from config.js */
const _CONFIG_PROJECTS = CONFIG_OPTIONS.CONFIG_PROJECTS;      /* Get from config.js */
const _FLYOUT_JS_FILE = document.currentScript.src;
const _FLYOUT_JS_DIR = _FLYOUT_JS_FILE.substring(0, _FLYOUT_JS_FILE.lastIndexOf('/'));
const _SERVER_ROOT = window.location.origin;

/**
 * Generates a target URL based on the selected language or version.
 *
 * This function modifies the current page path to reflect the requested language
 * or version and then verifies if the generated URL exists. If the URL is not
 * accessible, it returns a fallback URL to ensure a valid navigation path.
 *
 * @param {string} type - The type of change ('language' or 'version').
 * @param {string} selectedValue - The selected language code or version number.
 * @returns {Promise<string>} The generated target URL or a fallback URL if inaccessible.
 */
async function getTargetUrl(type, selectedValue) {
  const currentPath = window.location.pathname;
  let targetPath;

  /* Determine the target path based on the type (language or version). */
  if (type === "language") {
    targetPath = currentPath.replace(`/${_CURRENT_LANGUAGE}/`, `/${selectedValue}/`);
  } else if (type === "version") {
    targetPath = currentPath.replace(`/${_CURRENT_VERSION}/`, `/${selectedValue}/`);
  }

  /* If running locally (file:// protocol), use file-based path. Otherwise, use the server root URL. */
  const targetUrl = _IS_LOCAL
    ? `file://${targetPath}`
    : `${_SERVER_ROOT}${targetPath}`;

  /* If running locally, return the constructed file URL immediately. */
  if (_IS_LOCAL) return targetUrl;

  try {
    /* Send a HEAD request to check if the target URL exists. */
    const response = await fetch(targetUrl, { method: "HEAD" });
    /* If the response is OK (status 200), return the valid target URL. */
    if (response.ok) {
      return targetUrl;
    } else {
      console.warn("Target URL does not exist, using fallback URL:", targetUrl);
    }
  } catch (error) {
    /* Log any network or request errors. */
    console.error("Error checking target URL:", error);
  }

  /** If the target URL is not accessible, return a fallback URL.
      The fallback URL ensures the correct language and version are used. */
  return `${_FLYOUT_JS_DIR}/` +
         `${type === "language" ? selectedValue : _CURRENT_LANGUAGE}/` +
         `${type === "version" ? selectedValue : _CURRENT_VERSION}/index.html`;
}

/**
 * Creates and inserts a floating language and version selector into the page.
 *
 * This function dynamically generates an interactive flyout menu containing
 * selectable language options, version links, and project links. It also
 * manages the visibility state of the flyout using event listeners.
 */
function createFlyout() {
  const languageOptions = _CONFIG_LANGUAGES.filter(lang => lang[0] !== "newline").map(([code, name]) => {
    return `<option value="${code}" ${code === _CURRENT_LANGUAGE ? "selected" : ""}>${code} : ${name}</option>`;
  }).join("");

  const versionOptions = _CONFIG_VERSIONS.filter(version => version[0] !== "newline").map(([code, name]) => {
    return `<option value="${code}" ${code === _CURRENT_VERSION ? "selected" : ""}>${code} : ${name}</option>`;
  }).join("");

  const sortedProjects = _CONFIG_PROJECTS.map(([project, link]) => {
    return project === "newline"
      ? `<dd class="newline"></dd>`
      : `<dd class="options"><a href="${link}">${project}</a></dd>`;
  }).join("");

  const flyoutHTML = `
    <div class="ltd-flyout">
      <span class="ltd-flyout-header">
        <span class="ltd-flyout-header-icon">
          <img class="ltd-flyout-header-icon-img" src="${_FLYOUT_JS_DIR}/ltd-icon.svg" alt="icon">
        </span>
        <span class="ltd-flyout-header-label">
          l: ${_CURRENT_LANGUAGE} / v: ${_CURRENT_VERSION}
        </span>
      </span>
      <div class="ltd-flyout-divider ltd-flyout-closed"></div>
      <div class="ltd-flyout-content ltd-flyout-closed">
        <dl>
          <dt>Languages</dt>
          <dd class="select">
            <select id="language-select">
              ${languageOptions}
            </select>
          </dd>
        </dl>
        <dl>
          <dt>Versions</dt>
          <dd class="select">
            <select id="version-select">
              ${versionOptions}
            </select>
          </dd>
        </dl>
        <dl>
          <dt>Project Links</dt>
          ${sortedProjects}
        </dl>
      </div>
    </div>
  `;

  document.body.insertAdjacentHTML("beforeend", flyoutHTML);

  const flyout = document.querySelector(".ltd-flyout");
  const header = document.querySelector(".ltd-flyout-header");
  const icon = document.querySelector(".ltd-flyout-header-icon");
  const label = document.querySelector(".ltd-flyout-header-label");
  const dividers = document.querySelectorAll(".ltd-flyout-divider");
  const content = document.querySelector(".ltd-flyout-content");
  const languageSelect = document.getElementById("language-select");
  const versionSelect = document.getElementById("version-select");

  // Listen for the change event on the language dropdown
  languageSelect.addEventListener("change", async (event) => {
    const selectedLanguage = event.target.value;
    window.location.href = await getTargetUrl("language", selectedLanguage);
  });

  // Listen for the change event on the version dropdown
  versionSelect.addEventListener("change", async (event) => {
    const selectedVersion = event.target.value;
    window.location.href = await getTargetUrl("version", selectedVersion);
  });

  // Clicking the label toggles content and dividers
  label.addEventListener("click", (event) => {
    const isHidden = content.classList.toggle("ltd-flyout-closed");
    dividers.forEach(div => div.classList.toggle("ltd-flyout-closed", isHidden));
    event.stopPropagation();
  });

  // Clicking the icon toggles label visibility and resets all sections
  icon.addEventListener("click", (event) => {
    const labelHidden = label.classList.toggle("hidden");
    header.classList.toggle("ltd-icon-only", labelHidden);
    if (labelHidden) {
      content.classList.add("ltd-flyout-closed");
      dividers.forEach(div => div.classList.add("ltd-flyout-closed"));
    }
    event.stopPropagation();
  });

  // Clicking outside closes the content and dividers (label stays visible)
  document.addEventListener("click", (event) => {
    if (!flyout.contains(event.target)) {
      content.classList.add("ltd-flyout-closed");
      dividers.forEach(div => div.classList.add("ltd-flyout-closed"));
    }
  });
}

/**
 * Updates all language and version links after the page loads.
 *
 * This function selects all <a> elements containing language or version data attributes,
 * generates the appropriate URLs using `getTargetUrl`, and updates their `href` attributes.
 */
async function updateLinks() {
  // No need to update language links anymore
}

/**
 * Injects CSS styles for the floating language and version selector.
 *
 * This function creates a <style> element and appends it to the document head, defining
 * styles for the flyout menu, including its appearance, behavior, and interaction effects.
 */
function addStyles() {
  const css = `
    .ltd-flyout {
      color: #ffffff !important;
      background-color: #263238 !important;
      box-shadow: 0 4px 10px #000000 !important;
      font-family: Arial, sans-serif !important;
      font-size: 15px !important;
      line-height: 20px !important;
      position: fixed !important;
      right: 15px !important;
      bottom: 50px !important;
      z-index: 5000 !important;
      padding: 5px !important;
      border-radius: 5px !important;
      max-width: 320px !important;
    }

    .ltd-flyout-header {
      color: #27ae60 !important;
      background-color: #263238 !important;
      display: flex !important;
      padding: 5px !important;
      align-items: center !important;
      font-weight: bold !important;
      cursor: pointer !important;
    }

    .ltd-flyout-header.ltd-icon-only {
      justify-content: center !important;
      padding: 5px !important;
    }

    .ltd-flyout-header-icon {
      flex-shrink: 0 !important;
    }

    .ltd-flyout-header-icon-img {
      width: 25px !important;
      height: 25px !important;
      padding: 5px !important;
      display: block !important;
      box-sizing: content-box !important;
    }

    .ltd-flyout-header-label {
      padding: 5px !important;
      flex-grow: 1 !important;
      text-align: right !important;
    }

    .ltd-flyout-header-label.hidden {
      display: none !important;
    }

    .ltd-flyout-divider {
      background-color: #808080 !important;
      height: 1px !important;
      margin: 1px 5px !important;
    }

    .ltd-flyout-divider.ltd-flyout-closed {
      display: none !important;
    }

    .ltd-flyout-content {
      color: #ffffff !important;
      background-color: #263238 !important;
      padding: 10px !important;
      max-height: 450px !important;
      overflow-y: auto !important;
    }

    .ltd-flyout-content.ltd-flyout-closed {
      display: none !important;
    }

    .ltd-flyout-content dl {
      display: flex !important;
      flex-wrap: wrap !important;
      margin: 0 !important;
      padding: 0 !important;
    }

    .ltd-flyout-content dt {
      color: #808080 !important;
      width: 100% !important;
      font-weight: bold !important;
      text-align: left !important;
      padding: 0 !important;
    }

    .ltd-flyout-content dd {
      margin: 0 !important;
      padding: 0 !important;
    }

    .ltd-flyout-content dd::before {
      content: none !important;
    }

    .ltd-flyout-content dd.newline {
      flex-basis: 100% !important;
      height: 0 !important;
    }

    .ltd-flyout-content dd.select {
      display: block !important;
      margin-top: 5px !important;
      margin-bottom: 5px !important;
    }

    .ltd-flyout-content dd.select select {
      appearance: none !important; /* Remove default browser styling */
      color: #ffffff !important;
      background-color: #263238 !important;
      border: 1px solid #808080 !important;
      padding: 5px 30px 5px 10px !important; /* Add right padding for arrow */
      border-radius: 5px !important;
      font-size: inherit !important;
      line-height: inherit !important;
      width: 100% !important;
      box-sizing: border-box !important;
      box-shadow: none !important;
      background-image: url('data:image/svg+xml;utf8,<svg fill="%23ffffff" height="24" viewBox="0 0 24 24" width="24" xmlns="http://www.w3.org/2000/svg"><path d="M7 10l5 5 5-5z"/><path d="M0 0h24v24H0z" fill="none"/></svg>') !important; /* Custom arrow */
      background-repeat: no-repeat !important;
      background-position: right 5px center !important;
      cursor: pointer !important;
      max-width: none !important;
      min-width: 0px !important;
    }

    .ltd-flyout-content dd.select select:focus {
      outline: none !important;
      border-color: #27ae60 !important;
      box-shadow: 0 0 5px rgba(39, 174, 96, 0.5) !important;
    }

    .ltd-flyout-content dd.select select option {
      background-color: #263238 !important;
      color: #ffffff !important;
    }

    .ltd-flyout-content dd.select select option code {
      font-family: monospace !important;   /* Use monospace font */
      color: #ddd !important;              /* Adjust code color */
    }

    .ltd-flyout-content dd.options {
      display: flex !important;
      flex-wrap: wrap !important;
      gap: 5px !important;
      justify-content: flex-start !important;
    }

    .ltd-flyout-content dd.options a {
      color: #ffffff !important;
      background-color: #263238 !important;
      text-decoration: none !important;
      padding: 5px !important;
      border-radius: 5px !important;
      transition: background 0.3s !important;
      display: inline-block !important;
    }

    .ltd-flyout-content dd.options a:hover {
      color: #ffffff !important;
      background-color: #27ae60 !important;
    }

    .ltd-flyout-content dd.options a.selected {
      color: #ffffff !important;
      background-color: #27ae60 !important;
      font-weight: bold !important;
    }
  `;

  const styleTag = document.createElement("style");
  styleTag.innerHTML = css;
  document.head.appendChild(styleTag);
}

/**
 * Initializes the flyout menu and updates links when the page loads.
 *
 * This event listener waits for the DOM to be fully loaded before:
 * - Injecting the necessary CSS styles.
 * - Creating the floating language and version selector.
 * - Updating all language and version links with the correct URLs.
 */
document.addEventListener("DOMContentLoaded", async () => {
  addStyles();          /* Apply CSS styles for the flyout menu. */
  createFlyout();       /* Generate the language and version selector. */
  await updateLinks();  /* Update all <a> hrefs to reflect the correct URLs. */
});
