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

// Keeps a preview that opens above its link at Material's distance from the link, after the
// preview's height has changed from heightBefore. Material measures the preview's height when
// it appends the preview, before this script renders the math, and stores the preview's offset
// from the link in --md-tooltip-y, negative above the link. Rendering changes the height of a
// short preview, so the offset changes by the same amount. An element without a negative
// offset stays as it is.
var keepPreviewAboveItsLink = (function (preview, heightBefore) {
    var offset = parseFloat(preview.style.getPropertyValue('--md-tooltip-y'));
    if (offset < 0 && preview.offsetHeight !== heightBefore) {
      preview.style.setProperty('--md-tooltip-y',
                                (offset + heightBefore - preview.offsetHeight) + 'px');
    }
});

// Renders the math of an element that was appended to <body>. Rendering can change the
// element's height, so a hover preview is then moved back to its link.
var renderMathOfAddedElement = (function (element) {
    if (!element.querySelector('.arithmatex')) {
      return;
    }
    var heightBefore = element.offsetHeight;
    katexMath(element);
    keepPreviewAboveItsLink(element, heightBefore);
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
            var addedNode = records[i].addedNodes[j];
            if (addedNode.nodeType === Node.ELEMENT_NODE) {
              renderMathOfAddedElement(addedNode);
            }
          }
        }
      }).observe(document.body, {childList: true});
    }
  });
})();

}());
