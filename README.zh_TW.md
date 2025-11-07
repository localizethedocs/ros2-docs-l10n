<div align="center">
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://cdn.jsdelivr.net/gh/localizethedocs/static/mark/ros-white.svg">
  <source media="(prefers-color-scheme: light)" srcset="https://cdn.jsdelivr.net/gh/localizethedocs/static/mark/ros-dark.svg">
  <img height="100" src="https://cdn.jsdelivr.net/gh/localizethedocs/static/mark/ros-dark.svg">
</picture>
</div>

<div align="center">
<h1 id="localization-of-ros2-documentation">ROS 2 文件的在地化</h1>
</div>

<details><summary><strong>切換語言</strong></summary>
<p></p>
<ul>
  <li><a href="./README.md"><code>en_US</code> : English</a></li>
  <li><a href="./README.zh_CN.md"><code>zh_CN</code> : 简体中文</a></li>
  <li><a href="./README.zh_TW.md"><code>zh_TW</code> : 繁體中文</a></li>
</ul>
</details>

<h2 id="table-of-contents">目次表</h2>

<ul>
  <li><a href="#introduction">簡介</a></li>
  <li><a href="#progress-of-translations">翻譯進度</a></li>
  <li><a href="#preview-translations">預覽翻譯</a></li>
  <li><a href="#maintainers">維護者</a></li>
  <li><a href="#support-this-project">支援該專案</a></li>
  <li><a href="#sponsorship">贊助</a></li>
  <li><a href="#code-contributors">程式碼貢獻者</a></li>
  <li><a href="#financial-contributors">財務貢獻者</a></li>
  <li><a href="#licenses">授權條款</a></li>
</ul>

<h2 id="introduction"><a href="#table-of-contents">簡介</a></h2>

<div align="center"><a href="https://github.com/localizethedocs/ros2-docs-l10n/stargazers" title="GitHub Stars" target="_blank">
  <img alt="GitHub Stars" src="https://img.shields.io/github/stars/localizethedocs/ros2-docs-l10n.svg?logo=github&style=flat&color=blue" />
</a>
<a href="https://github.com/localizethedocs/ros2-docs-l10n/network/members" title="GitHub Forks" target="_blank">
  <img alt="GitHub Forks" src="https://img.shields.io/github/forks/localizethedocs/ros2-docs-l10n.svg?logo=github&style=flat&color=blue" />
</a>
<a href="https://github.com/localizethedocs/ros2-docs-l10n/graphs/contributors" title="GitHub Contributors" target="_blank">
  <img alt="GitHub Contributors" src="https://img.shields.io/github/contributors/localizethedocs/ros2-docs-l10n.svg?logo=github&style=flat&color=blue" />
</a>
<a href="https://github.com/localizethedocs/ros2-docs-l10n" title="GitHub Repo Size" target="_blank">
  <img alt="GitHub Repo Size" src="https://img.shields.io/github/repo-size/localizethedocs/ros2-docs-l10n?logo=github&style=flat&color=blue" />
</a>
<a href="https://github.com/localizethedocs/ros2-docs-l10n/issues" title="GitHub Open Issues" target="_blank">
  <img alt="GitHub Open Issues" src="https://img.shields.io/github/issues/localizethedocs/ros2-docs-l10n.svg?logo=github&style=flat&color=blue" />
</a>
<a href="https://github.com/localizethedocs/ros2-docs-l10n/pulls" title="GitHub Open Pull Requests" target="_blank">
  <img alt="GitHub Open Pull Requests" src="https://img.shields.io/github/issues-pr/localizethedocs/ros2-docs-l10n?logo=github&style=flat&color=blue" />
</a>
<a href="https://opensource.org/licenses/BSD-3-Clause" title="License: BSD-3-Clause" target="_blank">
  <img alt="License: BSD-3-Clause" src="https://img.shields.io/badge/License-BSD--3--Clause-blue.svg?logo=opensourceinitiative&style=flat&logoColor=white" />
</a>
<a href="http://creativecommons.org/licenses/by-sa/4.0/" title="License: CC-BY-SA-4.0" target="_blank">
  <img alt="License: CC-BY-SA-4.0" src="https://img.shields.io/badge/License-CC--BY--SA--4.0-blue.svg?logo=creativecommons&style=flat&logoColor=white" />
</a>
<a href="https://gitcode.com/localizethedocs/ros2-docs-l10n/stargazers" title="GitCode GStars" target="_blank">
  <img alt="GitCode GStars" src="https://gitcode.com/localizethedocs/ros2-docs-l10n/star/badge.svg" />
</a>
</div>

<p></p>

The goal of this project is to translate the ROS 2 documentation into multiple languages. Translations are contributed via the Crowdin platform, automatically synchronized with the GitHub repository, and can be previewed on GitHub Pages.

> [!NOTE]
> The translation is community-driven. If you find any inaccuracies, always refer to the <a href="https://docs.ros.org/">official documentation</a> or the <a href="https://github.com/ros2/ros2_documentation">source repository</a> of the upstream project for the most reliable information.

<div align="center">
<table>
  <thead>
    <tr>
      <th rowspan="1" colspan="2" align="center" style="text-align: center;"><div>專案連結</div></th>
    </tr>
  </thead>
  <tbody>
    <!-- Preview -->
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>預覽翻譯</div></th>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div><a href="https://projects.localizethedocs.org/ros2-docs-l10n" target="_blank">ros2-docs-l10n</a></div>
      </td>
    </tr>
    <!-- TMS -->
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>Crowdin</div></th>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div><a href="https://localizethedocs.crowdin.com/ros2-docs-l10n" target="_blank">ros2-docs-l10n</a></div>
      </td>
    </tr>
    <!-- Main -->
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>GitHub</div></th>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n" target="_blank">ros2-docs-l10n</a></div>
      </td>
    </tr>
    <!-- Mirror -->
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>GitCode</div></th>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div><a href="https://gitcode.com/localizethedocs/ros2-docs-l10n" target="_blank">ros2-docs-l10n</a></div>
      </td>
    </tr>
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>GitFlic</div></th>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div><a href="https://gitflic.ru/project/localizethedocs/ros2-docs-l10n" target="_blank">ros2-docs-l10n</a></div>
      </td>
    </tr>
  </tbody>
</table>
</div>

<div align="center">
<table>
  <thead>
    <tr>
      <th rowspan="1" colspan="2" align="center" style="text-align: center;"><div>GitHub Workflows</div></th>
    </tr>
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>Status</div></th>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>Description</div></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-sphinx-build-docs.yml" ><img alt="ci-sphinx-build-docs" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-sphinx-build-docs.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Build documents by Sphinx and upload build artifacts.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-sphinx-update-pot.yml" ><img alt="ci-sphinx-update-pot" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-sphinx-update-pot.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Update <code>.pot</code> files by Sphinx with <code>gettext</code> builder.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-gettext-update-po.yml" ><img alt="ci-gettext-update-po" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-gettext-update-po.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Update <code>.po</code> files from <code>.pot</code> files by Gettext toolkit.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-gettext-compendium.yml"><img alt="ci-gettext-compendium" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-gettext-compendium.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Update <code>.po</code> files from the compendium by Gettext toolkit.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-gettext-statistics.yml"><img alt="ci-gettext-statistics" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-gettext-statistics.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Update statistics information of <code>.po</code> files by Gettext toolkit.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-crowdin-upload-pot.yml"><img alt="ci-crowdin-upload-pot" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-crowdin-upload-pot.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Upload <code>.pot</code> files to Crowdin by its CLI tool.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-crowdin-download-po.yml"><img alt="ci-crowdin-download-po" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-crowdin-download-po.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Download <code>.po</code> files from Crowdin by its CLI tool.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-crowdin-update-docs.yml"><img alt="ci-crowdin-update-docs" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-crowdin-update-docs.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Update <code>docs</code> translations by Crowdin CLI tool.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-deploy-pages.yml"><img alt="ci-deploy-pages" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-deploy-pages.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Download and deploy build artifacts to <code>pages</code> branch.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-deploy-po-version.yml"><img alt="ci-deploy-po-version" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-deploy-po-version.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Deploy <code>.po</code> files to <code>po/${VERSION}</code> branch.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-update-license-year.yml"><img alt="ci-update-license-year" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-update-license-year.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Update the copyright year in LICENSE to the current year.</div>
      </td>
    </tr>
    <tr>
      <td rowspan="1" colspan="1" align="right" style="text-align: right;">
        <div><a href="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-update-submodule.yml"><img alt="ci-update-submodule" src="https://github.com/localizethedocs/ros2-docs-l10n/actions/workflows/ci-update-submodule.yml/badge.svg" /></a></div>
      </td>
      <td rowspan="1" colspan="1" align="left" style="text-align: left;">
        <div>Update each of the submodules to their latest commit.</div>
      </td>
    </tr>
  </tbody>
</table>
</div>

<h2 id="progress-of-translations"><a href="#table-of-contents">翻譯進度</a></h2>

<div align="center">
<table>
  <thead>
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>Locale</div></th>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div>
          <div>英文名稱</div>
          <div>在地名稱</div>
        </div>
      </th>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div>
          <div>已翻譯</div>
          <div>已校對</div>
        </div>
      </th>
    </tr>
  </thead>
  <tbody>
    <!-- zh_CN -->
    <tr>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;"><div><code>zh_CN</code></div></td>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div>
          <div>Simplified Chinese</div>
          <div>简体中文</div>
        </div>
      </td>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div>
          <div><a href="https://localizethedocs.crowdin.com/ros2-docs-l10n/zh-CN" title="zh-CN translation" target="_blank"><img alt="zh-CN translation" src="https://img.shields.io/badge/dynamic/json?color=blue&label=zh-CN&style=flat&logo=crowdin&query=%24.progress.0.data.translationProgress&url=https%3A%2F%2Fbadges.awesome-crowdin.com%2Fstats-200032568-5.json" /></a></div>
          <div><a href="https://localizethedocs.crowdin.com/ros2-docs-l10n/zh-CN" title="zh-CN proofreading" target="_blank"><img alt="zh-CN proofreading" src="https://img.shields.io/badge/dynamic/json?color=green&label=zh-CN&style=flat&logo=crowdin&query=%24.progress.0.data.approvalProgress&url=https%3A%2F%2Fbadges.awesome-crowdin.com%2Fstats-200032568-5.json" /></a></div>
        </div>
      </td>
    </tr>
    <!-- zh_TW -->
    <tr>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;"><div><code>zh_TW</code></div></td>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div>
          <div>Traditional Chinese</div>
          <div>繁體中文</div>
        </div>
      </td>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;">
        <div>
          <div><a href="https://localizethedocs.crowdin.com/ros2-docs-l10n/zh-TW" title="zh-TW translation" target="_blank"><img alt="zh-TW translation" src="https://img.shields.io/badge/dynamic/json?color=blue&label=zh-TW&style=flat&logo=crowdin&query=%24.progress.1.data.translationProgress&url=https%3A%2F%2Fbadges.awesome-crowdin.com%2Fstats-200032568-5.json" /></a></div>
          <div><a href="https://localizethedocs.crowdin.com/ros2-docs-l10n/zh-TW" title="zh-TW proofreading" target="_blank"><img alt="zh-TW proofreading" src="https://img.shields.io/badge/dynamic/json?color=green&label=zh-TW&style=flat&logo=crowdin&query=%24.progress.1.data.approvalProgress&url=https%3A%2F%2Fbadges.awesome-crowdin.com%2Fstats-200032568-5.json" /></a></div>
        </div>
      </td>
    </tr>
  </tbody>
</table>
</div>

<div align="center"><a href="https://localizethedocs.crowdin.com/ros2-docs-l10n"><img src="https://badges.awesome-crowdin.com/translation-200032568-5.png"></a>
</div>

<h2 id="preview-translations"><a href="#table-of-contents">預覽翻譯</a></h2>

You can preview translations in <a href="https://projects.localizethedocs.org/ros2-docs-l10n">GitHub Pages</a>:

<div align="center">
<table>
  <thead>
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>語言</div></th>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>GitHub 網頁</div></th>
    </tr>
  </thead>
  <tbody>
    <!-- en_US -->
    <tr>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;"><div><code>en_US</code></div></td>
      <td rowspan="1" colspan="1" align="left"   style="text-align: left;  ">
        <div><a href="https://projects.localizethedocs.org/ros2-docs-l10n/en-us">https://projects.localizethedocs.org/ros2-docs-l10n/en-us</a></div>
      </td>
    </tr>
    <!-- zh_CN -->
    <tr>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;"><div><code>zh_CN</code></div></td>
      <td rowspan="1" colspan="1" align="left"   style="text-align: left;  ">
        <div><a href="https://projects.localizethedocs.org/ros2-docs-l10n/zh-cn">https://projects.localizethedocs.org/ros2-docs-l10n/zh-cn</a></div>
      </td>
    </tr>
    <!-- zh_TW -->
    <tr>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;"><div><code>zh_TW</code></div></td>
      <td rowspan="1" colspan="1" align="left"   style="text-align: left;  ">
        <div><a href="https://projects.localizethedocs.org/ros2-docs-l10n/zh-tw">https://projects.localizethedocs.org/ros2-docs-l10n/zh-tw</a></div>
      </td>
    </tr>
  </tbody>
</table>
</div>

Alternatively, you can preview translations locally by running the following commands:

```bash
# Please make sure the following prerequisites are installed:
# CMake (3.25~), Git, Conda, Gettext
LANGUAGE=zh_TW
VERSION=rolling
git clone --recurse-submodule https://github.com/localizethedocs/ros2-docs-l10n.git
cd ros2-docs-l10n
cmake --preset ${LANGUAGE} -DVERSION=${VERSION}
cmake --build out/build/${LANGUAGE}
```

You can switch to other available languages and versions by the flyout navigation menu.

<h2 id="maintainers"><a href="#table-of-contents">維護者</a></h2>

This project is currently maintained by:

<div align="center">
<table>
  <thead>
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>Name</div></th>
      <th rowspan="1" colspan="1" align="center" style="text-align: center;"><div>GitHub Username</div></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;"><div>Haowei Hsu</div></td>
      <td rowspan="1" colspan="1" align="center" style="text-align: center;"><div>@hwhsu1231</div></td>
    </tr>
  </tbody>
</table>
</div>

<h2 id="support-this-project"><a href="#table-of-contents">Support this Project</a></h2>

You can contribute and support this project by doing any of the following:

<ul>
  <li>Star the project on GitHub or other mirrors.</li>
  <li>Submit issues on <a href="https://github.com/localizethedocs/ros2-docs-l10n/issues">GitHub</a> if you found any bug.</li>
  <li>Submit pull requests on <a href="https://github.com/localizethedocs/ros2-docs-l10n/pulls">GitHub</a> if you know how to fix the bug.</li>
  <li>Contribute your translations on <a href="https://localizethedocs.crowdin.com/ros2-docs-l10n">Crowdin</a>.</li>
  <li>If you love this kind of l10n projects, please consider <a href="#sponsorship">sponsorship</a>.</li>
</ul>

It will facilitate the creation and ongoing maintenance of l10n projects for open-source documentation.

<h2 id="sponsorship"><a href="#table-of-contents">Sponsorship</a></h2>

You can sponsor the projects through the following platforms:

<div align="center">
<table>
  <tbody>
    <!-- Row 1 -->
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center; width: 50%;"><div>Patreon</div></th>
      <th rowspan="1" colspan="1" align="center" style="text-align: center; width: 50%;"><div>Afdian</div></th>
    </tr>
    <!-- Row 2 -->
    <tr>
      <td rowspan="1" colspan="1" align="center" style="text-align: center; width: 50%;" >
        <div><a href="https://www.patreon.com/localizethedocs" target="_blank">
          <img width="200" src="https://cdn.jsdelivr.net/gh/localizethedocs/static/button/sponsor-patreon.png" alt="Patreon" />
        </a></div>
      </td>
      <td rowspan="1" colspan="1" align="center" style="text-align: center; width: 50%;">
        <div><a href="https://www.afdian.com/a/localizethedocs" target="_blank">
          <img width="200" src="https://cdn.jsdelivr.net/gh/localizethedocs/static/button/sponsor-afdian.png" alt="Afdian" />
        </a></div>
      </td>
    </tr>
  </tbody>
</table>
</div>

Alternatively, you can sponsor the maintainers directly through the following platforms:

<div align="center">
<table>
  <tbody>
    <tr>
      <th rowspan="1" colspan="2" align="center" style="text-align: center; width: 100%;"><div>Haowei Hsu</div></th>
    </tr>
    <!-- Row 1 -->
    <tr>
      <th rowspan="1" colspan="1" align="center" style="text-align: center; width: 50%;"><div>Patreon</div></th>
      <th rowspan="1" colspan="1" align="center" style="text-align: center; width: 50%;"><div>Afdian</div></th>
    </tr>
    <!-- Row 2 -->
    <tr>
      <td rowspan="1" colspan="1" align="center" style="text-align: center; width: 50%;" >
        <div><a href="https://www.patreon.com/hwhsu1231" target="_blank">
          <img width="200" src="https://cdn.jsdelivr.net/gh/localizethedocs/static/button/sponsor-patreon.png" alt="Patreon" />
        </a></div>
      </td>
      <td rowspan="1" colspan="1" align="center" style="text-align: center; width: 50%;">
        <div><a href="https://www.afdian.com/a/hwhsu1231" target="_blank">
          <img width="200" src="https://cdn.jsdelivr.net/gh/localizethedocs/static/button/sponsor-afdian.png" alt="Afdian" />
        </a></div>
      </td>
    </tr>
  </tbody>
</table>
</div>

<h2 id="code-contributors"><a href="#table-of-contents">Code Contributors</a></h2>

<div align="center">
<img width="100%" alt="GitHub Contributors SVG" src="https://contrib.nn.ci/api?repo=localizethedocs/ros2-docs-l10n&no_bot=true" />
</div>

<h2 id="financial-contributors"><a href="#table-of-contents">Financial Contributors</a></h2>

<div align="center">
<h3 id="localize-the-docs">Localize The Docs</h3>
</div>

<div align="center">
<img width="100%" alt="Localize The Docs" src="https://cdn.jsdelivr.net/gh/localizethedocs/sponsors/static/sponsors.svg" />
</div>

<div align="center">
<h3 id="haowei-hsu">Haowei Hsu</h3>
</div>

<div align="center">
<img width="100%" alt="Haowei Hsu" src="https://cdn.jsdelivr.net/gh/hwhsu1231/sponsors/static/sponsors.svg" />
</div>

<h2 id="licenses"><a href="#table-of-contents">Licenses</a></h2>

All original and translated text in the <code>.pot</code> and<code>.po</code> files stored in the <code>l10n</code> branch is made available under the same license as <a href="https://github.com/ros2/ros2_documentation">the upstream project</a>.

All CMake script files (e.g., <code>CMakeLists.txt</code>, <code>CMakePresets.json</code>, and <code>cmake/\*\*/\*</code> files), GitHub workflow files (e.g., <code>.github/workflows/\*.yml</code> files), and configuration files (e.g., <code>versions.json</code>, <code>languages.json</code>, <code>crowdin.yml</code>, and so on) of this project are licensed under <a href="https://opensource.org/license/bsd-3-clause/">3-Clause BSD</a> license. See <a href="./LICENSE-BSD">LICENSE-BSD</a> for details.

All document files (e.g., <code>README.md</code>) of this project are licensed under <a href="https://creativecommons.org/licenses/by-sa/4.0/">Creative Commons Attribution-ShareAlike 4.0 International</a> license. See <a href="./LICENSE-CC">LICENSE-CC</a> for details.

<a href="http://creativecommons.org/licenses/by-sa/4.0/" title="CC BY SA 4.0" target="_blank">
  <img alt="CC BY SA 4.0" src="https://i.creativecommons.org/l/by-sa/4.0/88x31.png"/>
</a>
