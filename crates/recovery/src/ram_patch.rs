//! Checked changes to a startup RAM image, applied before its CRC is taken.
use crate::image::{self, Result, extent, require, word};
use serde::Deserialize;
use std::path::Path;

/// One 32-bit word inside the stock image that must hold `old`.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct WordEdit {
    pub address: u64,
    pub old: u32,
    pub new: u32,
}

/// Bytes placed past the end of the stock image.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Payload {
    pub address: u64,
    pub bytes: Vec<u8>,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct RamPatches {
    pub words: Vec<WordEdit>,
    pub payloads: Vec<Payload>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PayloadFile {
    address: u64,
    /// Relative to the patch file's directory.
    file: String,
    sha256: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PatchFile {
    #[serde(default)]
    words: Vec<WordEdit>,
    #[serde(default)]
    payloads: Vec<PayloadFile>,
}

impl RamPatches {
    pub fn is_empty(&self) -> bool {
        self.words.is_empty() && self.payloads.is_empty()
    }

    /// Reads a patch file whose payload bytes live in hash-checked files
    /// beside it.
    pub fn read(path: &Path) -> Result<Self> {
        let text = std::fs::read_to_string(path)
            .map_err(|e| image::Error::Invalid(format!("RAM patch file: {e}")))?;
        let file: PatchFile = serde_json::from_str(&text)
            .map_err(|e| image::Error::Invalid(format!("RAM patch file: {e}")))?;
        let directory = path.parent().unwrap_or(Path::new("."));
        let mut payloads = Vec::new();
        for row in file.payloads {
            require(
                Path::new(&row.file)
                    .components()
                    .all(|c| matches!(c, std::path::Component::Normal(_))),
                "RAM payload path leaves the patch directory",
            )?;
            let bytes = std::fs::read(directory.join(&row.file))
                .map_err(|e| image::Error::Invalid(format!("RAM payload {}: {e}", row.file)))?;
            image::hash(&bytes, &row.sha256)?;
            payloads.push(Payload {
                address: row.address,
                bytes,
            });
        }
        Ok(Self {
            words: file.words,
            payloads,
        })
    }

    /// Applies every edit to `ram` or none. Words must lie inside the stock
    /// image and hold their old value; payloads must lie past its end, where
    /// the image grows with zeros.
    pub fn apply(&self, ram: &mut Vec<u8>) -> Result<()> {
        let stock = ram.len() as u64;
        let mut words: Vec<u64> = self.words.iter().map(|w| w.address).collect();
        words.sort_unstable();
        require(
            words.windows(2).all(|pair| pair[0] != pair[1]),
            "RAM patch edits one word twice",
        )?;
        for edit in &self.words {
            require(
                edit.address.is_multiple_of(4) && edit.address + 4 <= stock,
                "RAM word edit lies outside the stock image",
            )?;
            require(
                word(ram, edit.address as usize)? == edit.old,
                "RAM word edit does not match the stock word",
            )?;
        }
        let mut spans: Vec<(u64, u64)> = Vec::new();
        for payload in &self.payloads {
            let end = payload.address.checked_add(payload.bytes.len() as u64);
            require(
                !payload.bytes.is_empty()
                    && payload.address >= stock
                    && end.is_some_and(|end| end < 1 << 30),
                "RAM payload must lie past the stock image and below 1 GiB",
            )?;
            spans.push((payload.address, end.unwrap_or(u64::MAX)));
        }
        spans.sort_unstable();
        require(
            spans.windows(2).all(|pair| pair[0].1 <= pair[1].0),
            "RAM payloads overlap",
        )?;
        let end = spans.last().map_or(stock, |span| span.1);
        ram.resize(end as usize, 0);
        for edit in &self.words {
            ram[edit.address as usize..edit.address as usize + 4]
                .copy_from_slice(&edit.new.to_le_bytes());
        }
        for payload in &self.payloads {
            let at = payload.address as usize;
            extent(ram, at, payload.bytes.len())?;
            ram[at..at + payload.bytes.len()].copy_from_slice(&payload.bytes);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn image() -> Vec<u8> {
        (0..64u8).collect()
    }

    #[test]
    fn words_and_payloads_apply_and_grow_the_image() {
        let mut ram = image();
        let patches = RamPatches {
            words: vec![WordEdit {
                address: 8,
                old: 0x0b0a0908,
                new: 0x94000000,
            }],
            payloads: vec![Payload {
                address: 80,
                bytes: vec![1, 2, 3],
            }],
        };
        patches.apply(&mut ram).unwrap();
        assert_eq!(ram.len(), 83);
        assert_eq!(word(&ram, 8).unwrap(), 0x94000000);
        assert!(ram[64..80].iter().all(|v| *v == 0));
        assert_eq!(&ram[80..], [1, 2, 3]);
    }

    #[test]
    fn mismatched_or_misplaced_edits_leave_the_image_unchanged() {
        let good = WordEdit {
            address: 8,
            old: 0x0b0a0908,
            new: 1,
        };
        for patches in [
            RamPatches {
                words: vec![WordEdit {
                    old: 0,
                    ..good.clone()
                }],
                payloads: vec![],
            },
            RamPatches {
                words: vec![WordEdit {
                    address: 6,
                    ..good.clone()
                }],
                payloads: vec![],
            },
            RamPatches {
                words: vec![WordEdit {
                    address: 64,
                    ..good.clone()
                }],
                payloads: vec![],
            },
            RamPatches {
                words: vec![good.clone(), good.clone()],
                payloads: vec![],
            },
            RamPatches {
                words: vec![good.clone()],
                payloads: vec![Payload {
                    address: 60,
                    bytes: vec![1],
                }],
            },
            RamPatches {
                words: vec![],
                payloads: vec![
                    Payload {
                        address: 64,
                        bytes: vec![1, 2],
                    },
                    Payload {
                        address: 65,
                        bytes: vec![3],
                    },
                ],
            },
            RamPatches {
                words: vec![],
                payloads: vec![Payload {
                    address: 64,
                    bytes: vec![],
                }],
            },
            RamPatches {
                words: vec![],
                payloads: vec![Payload {
                    address: 1 << 30,
                    bytes: vec![1],
                }],
            },
        ] {
            let mut ram = image();
            assert!(patches.apply(&mut ram).is_err(), "{patches:?}");
            assert_eq!(ram, image());
        }
    }

    #[test]
    fn patch_files_read_hash_checked_payloads_beside_them() {
        let directory = std::env::temp_dir().join(format!("ram-patch-{}", std::process::id()));
        std::fs::create_dir_all(&directory).unwrap();
        std::fs::write(directory.join("payload.bin"), [7u8; 200]).unwrap();
        let sha = fujimod_firmware::sha256(&[7u8; 200]);
        let write = |text: String| {
            std::fs::write(directory.join("patches.json"), text).unwrap();
            RamPatches::read(&directory.join("patches.json"))
        };
        let read = write(format!(
            r#"{{"words":[{{"address":8,"old":1,"new":2}}],"payloads":[{{"address":4096,"file":"payload.bin","sha256":"{sha}"}}]}}"#
        ))
        .unwrap();
        assert_eq!(
            read.words,
            vec![WordEdit {
                address: 8,
                old: 1,
                new: 2
            }]
        );
        assert_eq!(
            read.payloads,
            vec![Payload {
                address: 4096,
                bytes: vec![7; 200]
            }]
        );
        let zero = "0".repeat(64);
        assert!(
            write(format!(
                r#"{{"payloads":[{{"address":4096,"file":"payload.bin","sha256":"{zero}"}}]}}"#
            ))
            .is_err()
        );
        assert!(
            write(format!(
                r#"{{"payloads":[{{"address":4096,"file":"../payload.bin","sha256":"{sha}"}}]}}"#
            ))
            .is_err()
        );
        assert!(write(r#"{"words":[],"extra":1}"#.into()).is_err());
        std::fs::remove_dir_all(&directory).unwrap();
    }
}
