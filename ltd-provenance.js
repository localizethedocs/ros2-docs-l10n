"use strict";

/**
 * Creates and injects a dropdown-style provenance dropdown box into the page.
 * The dropdown box is collapsed by default and can be expanded by clicking on it.
 */
function createProvenanceDropdown() {
  // 1. Define the HTML and CSS as template literals.
  const flyoutHTML = `
    <strong>
      <details class="provenance-dropdown">
        <summary class="provenance-summary">Provenance</summary>
        <div class="provenance-content">
          The translation is community-driven. If you find any inaccuracies, always refer to the <a href="https://docs.ros.org">official documentation</a> or the <a href="https://github.com/ros2/ros2_documentation">source repository</a> of the upstream project for the most reliable information.
        </div>
      </details>
    </strong>
  `;

  const flyoutCSS = `
    /* Styles for the main dropdown container */
    .provenance-dropdown {
      background-color: #d1ecf1; /* Light blue background */
      border: 1px solid #c8e5e8; /* Light blue border */
      color: #0c5460; /* Dark blue text */
      padding: 15px;
      margin-bottom: 20px;
      border-radius: 5px;
      box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
    }

    /* Styles for the summary (the clickable header) */
    .provenance-summary {
      cursor: pointer;
      list-style: none; /* Hide the default dropdown arrow */
    }

    /* Re-add a custom arrow/icon for the dropdown */
    .provenance-summary::before {
      content: "▼"; /* Downward arrow */
      display: inline-block;
      margin-right: 10px;
      transition: transform 0.2s ease;
    }

    /* Rotate the arrow when the dropdown is open */
    .provenance-dropdown[open] .provenance-summary::before {
      transform: rotate(-180deg);
    }

    /* Styling for the content of the dropdown */
    .provenance-content {
      line-height: 24px;
      padding-top: 10px; /* Add some space between the summary and content */
    }

    /* Remove padding and margin from summary */
    .provenance-summary::-webkit-details-marker,
    .provenance-summary::marker {
      display: none;
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

  const mainContent = document.querySelector('');
  if (mainContent) {
    mainContent.prepend(dropdownBox);
  } else {
    document.body.prepend(dropdownBox);
  }
}

document.addEventListener("DOMContentLoaded", createProvenanceDropdown);
