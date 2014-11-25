for(header, INSTALL_HEADERS) {
  path = $${INSTALL_PREFIX}/$${dirname(header)}
  eval(headers_$${header}.files += $$header)
  eval(headers_$${header}.path = $$path)
  eval(INSTALLS *= headers_$${header})
}
