"use strict";

/**
 * Creates and injects a dropdown-style provenance dropdown box into the page.
 * The dropdown box is collapsed by default and can be expanded by clicking on it.
 */
function createProvenanceDropdown() {
  // 1. Define the HTML and CSS as template literals.
  const flyoutHTML = `
    <details class="provenance-dropdown">
      <summary class="provenance-summary"><strong>Provenance</strong></summary>
      <div class="provenance-content">
        The translation is <strong>unofficial</strong> and <strong>community-driven</strong>. If you find any inaccuracies, always refer to the <a href="https://docs.ros.org">official documentation</a> or the <a href="https://github.com/ros2/ros2_documentation">source repository</a> of the upstream project for the most reliable information.
      </div>
    </details>
  `;

  const flyoutCSS = `
    /* Styles for the main dropdown container */
    details.provenance-dropdown {
      color: #ffffff !important;
      background-color: #263238 !important;
      border: none !important;
      box-shadow: 0 4px 10px #000000 !important;
      font-family: Arial, sans-serif !important;
      font-size: 15px !important;
      line-height: 20px !important;
      padding: 15px !important;
      margin: 10px 0 10px 0 !important;
      border-radius: 5px !important;
    }

    /* Styles for the summary (the clickable header) */
    .provenance-summary {
      color: #27ae60 !important;
      background-color: #263238 !important;
      cursor: pointer !important;
      list-style: none !important; /* Hide the default dropdown arrow */
      font-weight: bold !important;
    }

    /* Re-add a custom arrow/icon for the dropdown */
    .provenance-summary::before {
      content: "▼" !important; /* Downward arrow */
      color: #27ae60 !important;
      display: inline-block !important;
      margin-right: 10px !important;
      transition: transform 0.2s ease !important;
    }

    /* Rotate the arrow when the dropdown is open */
    .provenance-dropdown[open] .provenance-summary::before {
      transform: rotate(-180deg) !important;
    }

    /* Styling for the content of the dropdown */
    .provenance-content {
      color: #ffffff !important;
      background-color: #263238 !important;
      line-height: 24px !important;
      padding-top: 10px !important; /* Add some space between the summary and content */
    }

    .provenance-content a {
      color: #27ae60 !important;
      background-color: #263238 !important;
      text-decoration: underline !important;
      transition: color 0.3s !important;
      display: inline-block !important;
    }

    .provenance-content a:hover {
      color: #ffffff !important;
      background-color: #263238 !important;
    }

    /* Remove padding and margin from summary */
    .provenance-summary::-webkit-details-marker,
    .provenance-summary::marker {
      display: none !important;
    }
  `;

  // 2. Create and append the <style> element to the document head.
  const styleTag = document.createElement('style');
  styleTag.innerHTML = flyoutCSS;
  document.head.appendChild(styleTag);

  // 3. Create the HTML element and insert it into the page.
  const tempDiv = document.createElement('div');
  tempDiv.innerHTML = flyoutHTML.trim();
  const dropdownBox = tempDiv.firstChild;

  const mainContent = document.querySelector('div[role="main"]');
  if (mainContent) {
    mainContent.prepend(dropdownBox);
  } else {
    document.body.prepend(dropdownBox);
  }
}

document.addEventListener("DOMContentLoaded", createProvenanceDropdown);
