(function () {
'use strict';

// Renders the math inside root. pymdownx.arithmatex writes math as elements with the class
// "arithmatex" whose text is the source, \(...\) or \[...\]. A rendered element no longer
// starts with these, so a second call leaves it as it is.
var katexMath = (function (root) {
    var maths = root.querySelectorAll('.arithmatex'),
        tex;

    for (var i = 0; i < maths.length; i++) {
      tex = maths[i].textContent || maths[i].innerText;
      if (tex.startsWith('\\(') && tex.endsWith('\\)')) {
        katex.render(tex.slice(2, -2), maths[i], {'displayMode': false, 'throwOnError': false});
      } else if (tex.startsWith('\\[') && tex.endsWith('\\]')) {
        katex.render(tex.slice(2, -2), maths[i], {'displayMode': true, 'throwOnError': false});
      }
    }
});

// Renders the math of an element that was appended to <body>, and keeps a preview that opens
// above its link at Material's distance from the link. Material measures the preview's height
// when it appends the preview, before this script renders the math, and stores the preview's
// offset from the link in --md-tooltip-y, negative above the link. Rendering changes the height
// of a short preview, so the offset changes by the same amount.
var renderAdded = (function (element) {
    if (!element.querySelector('.arithmatex')) {
      return;
    }
    var height = element.offsetHeight;
    katexMath(element);
    var offset = parseFloat(element.style.getPropertyValue('--md-tooltip-y'));
    if (offset < 0 && element.offsetHeight !== height) {
      element.style.setProperty('--md-tooltip-y', (offset + height - element.offsetHeight) + 'px');
    }
});

(function () {
  var onReady = function onReady(fn) {
    if (document.addEventListener) {
      document.addEventListener("DOMContentLoaded", fn);
    } else {
      document.attachEvent("onreadystatechange", function () {
        if (document.readyState === "interactive") {
          fn();
        }
      });
    }
  };

  onReady(function () {
    if (typeof katex !== "undefined") {
      katexMath(document);

      // Hover previews of links (navigation.instant.preview in mkdocs.yml): Material fetches the
      // target page, where the math is still source text, and appends the previewed section to
      // <body> when a visitor hovers the link. The observer renders the math of every element
      // that is appended to <body> after the page has loaded.
      new MutationObserver(function (records) {
        for (var i = 0; i < records.length; i++) {
          for (var j = 0; j < records[i].addedNodes.length; j++) {
            if (records[i].addedNodes[j].nodeType === Node.ELEMENT_NODE) {
              renderAdded(records[i].addedNodes[j]);
            }
          }
        }
      }).observe(document.body, {childList: true});
    }
  });
})();

}());
