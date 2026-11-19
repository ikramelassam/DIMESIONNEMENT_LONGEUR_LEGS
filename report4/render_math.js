// Pre-rend les formules $..$ et $$..$$ avec KaTeX. Usage: node render_math.js in.html out.html <katex_dir>
const fs=require('fs');const katex=require(process.argv[4]+'/katex');
let h=fs.readFileSync(process.argv[2],'utf8');
const R=(t,d)=>katex.renderToString(t.replace(/&lt;/g,'<').replace(/&gt;/g,'>').replace(/&amp;/g,'&'),{displayMode:d,throwOnError:true,strict:'ignore'});
h=h.replace(/\$\$([\s\S]+?)\$\$/g,(m,t)=>'<div class="eq">'+R(t,true)+'</div>');
h=h.replace(/\$([^$\n]+?)\$/g,(m,t)=>R(t,false));
fs.writeFileSync(process.argv[3],h);
